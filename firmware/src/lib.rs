#![no_std]
#![feature(type_alias_impl_trait)]

use core::fmt::{Display, Write};

use arrayvec::ArrayVec;
use embassy_net::{tcp::TcpSocket, Stack};
use esp_println::println;
use esp_wifi::wifi::{WifiDevice, WifiStaDevice};
use fixed::{traits::ToFixed, types::I20F12};
use hal::Rtc;
use time::{Date, Month, OffsetDateTime, PrimitiveDateTime};

// TODO: pass these as parameters instead of hard-coding them into the lib?
const INFLUX_TOKEN: &str = env!("INFLUX_TOKEN");
const INFLUX_SERVER: &str = env!("INFLUX_SERVER");

pub mod analog;

#[derive(Copy, Clone)]
pub struct Voltage {
    pub mv: I20F12,
}

impl Voltage {
    pub const ZERO: Voltage = Voltage { mv: I20F12::ZERO };

    pub fn mv(&self) -> I20F12 {
        self.mv
    }

    pub fn v(&self) -> I20F12 {
        self.mv / 1000
    }

    pub fn from_v<N: ToFixed>(v: N) -> Self {
        Self {
            mv: v.to_fixed::<I20F12>() * 1000,
        }
    }
}

impl Display for Voltage {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{} mV", self.mv)
    }
}

impl<Rhs: ToFixed> core::ops::Div<Rhs> for Voltage {
    type Output = Voltage;

    fn div(self, rhs: Rhs) -> Self::Output {
        Voltage {
            mv: self.mv / rhs.to_fixed::<I20F12>(),
        }
    }
}

#[derive(Copy, Clone, Default)]
pub struct Temperature {
    pub deg: I20F12,
}

impl Temperature {
    pub fn from_tmp36_voltage(v: Voltage) -> Self {
        Self {
            deg: (v.mv - I20F12::from_num(500)) / 10,
        }
    }

    pub fn from_degrees<N: ToFixed>(deg: N) -> Self {
        Self {
            deg: deg.to_fixed(),
        }
    }

    pub fn degrees(&self) -> I20F12 {
        self.deg
    }
}

#[derive(Clone)]
pub struct Measurement {
    pub temperature: Temperature,
    pub battery: Option<Voltage>,
    pub rtc_ms: u64,
}

pub enum GetTimeError {
    Write(SocketWriterError),
    Read(SocketReaderError),
    NoTime,
}

impl From<SocketWriterError> for GetTimeError {
    fn from(w: SocketWriterError) -> Self {
        Self::Write(w)
    }
}

impl From<SocketReaderError> for GetTimeError {
    fn from(w: SocketReaderError) -> Self {
        Self::Read(w)
    }
}

const DAY_NAMES: &[&str] = &["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"];
const MONTH_NAMES: &[&str] = &[
    "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
];

#[macro_export]
macro_rules! singleton {
    ($val:expr, $typ:ty) => {{
        static STATIC_CELL: StaticCell<$typ> = StaticCell::new();
        STATIC_CELL.init($val)
    }};
}

fn parse_header_date(header: &str) -> Option<PrimitiveDateTime> {
    let rest = header.strip_prefix("Date: ")?;
    let (day, rest) = rest.split_at(3);
    if !DAY_NAMES.contains(&day) {
        return None;
    }
    let rest = rest.strip_prefix(", ")?;
    let (day, rest) = rest.split_at(2);
    let rest = rest.strip_prefix(' ')?;
    let (month, rest) = rest.split_at(3);
    let rest = rest.strip_prefix(' ')?;
    let (year, rest) = rest.split_at(4);
    let rest = rest.strip_prefix(' ')?;
    let (hour, rest) = rest.split_at(2);
    let rest = rest.strip_prefix(':')?;
    let (minute, rest) = rest.split_at(2);
    let rest = rest.strip_prefix(':')?;
    let (second, _rest) = rest.split_at(2);

    let month = Month::try_from(MONTH_NAMES.iter().position(|m| m == &month)? as u8 + 1).unwrap();

    Date::from_calendar_date(year.parse().ok()?, month, day.parse().ok()?)
        .ok()?
        .with_hms(
            hour.parse().ok()?,
            minute.parse().ok()?,
            second.parse().ok()?,
        )
        .ok()
}

#[derive(Default)]
pub struct WriteBuf<const S: usize> {
    buf: ArrayVec<u8, S>,
}

impl<const S: usize> core::fmt::Write for WriteBuf<S> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.buf
            .try_extend_from_slice(s.as_bytes())
            .map_err(|_| core::fmt::Error)
    }
}

impl<const S: usize> WriteBuf<S> {
    pub fn len(&self) -> usize {
        self.buf.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }

    pub fn as_bytes(&self) -> &[u8] {
        self.buf.as_slice()
    }

    pub fn clear(&mut self) {
        self.buf.clear()
    }

    pub async fn write_http_chunk<const CAP: usize>(&mut self, w: &mut SocketWriter<'_, '_, CAP>) {
        write!(w, "{:X}\r\n", self.buf.len()).await.unwrap();
        w.write_bytes(self.as_bytes()).await.unwrap();
        write!(w, "\r\n").await.unwrap();
        self.clear();
    }
}

pub struct SocketWriter<'a, 'b: 'a, const CAP: usize> {
    socket: &'a mut TcpSocket<'b>,
    buf: WriteBuf<CAP>,
}

#[derive(Debug)]
pub enum SocketWriterError {
    TooLong,
    Tcp(embassy_net::tcp::Error),
}

impl<'a, 'b: 'a, const CAP: usize> SocketWriter<'a, 'b, CAP> {
    pub fn new(socket: &'a mut TcpSocket<'b>) -> Self {
        Self {
            socket,
            buf: Default::default(),
        }
    }

    pub async fn write_fmt(
        &mut self,
        args: core::fmt::Arguments<'_>,
    ) -> Result<(), SocketWriterError> {
        let old_size = self.buf.buf.len();
        if self.buf.write_fmt(args).is_ok() {
            return Ok(());
        }

        self.buf.buf.truncate(old_size);
        self.flush_buf().await?;

        self.buf
            .write_fmt(args)
            .map_err(|_| SocketWriterError::TooLong)
    }

    async fn flush_buf(&mut self) -> Result<(), SocketWriterError> {
        embedded_io_async::Write::write_all(&mut self.socket, self.buf.buf.as_slice())
            .await
            .map_err(SocketWriterError::Tcp)?;
        self.buf.buf.clear();
        Ok(())
    }

    pub async fn flush(&mut self) -> Result<(), SocketWriterError> {
        self.flush_buf().await?;
        self.socket.flush().await.map_err(SocketWriterError::Tcp)?;
        Ok(())
    }

    pub async fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), SocketWriterError> {
        self.flush_buf().await?;
        embedded_io_async::Write::write_all(&mut self.socket, bytes)
            .await
            .map_err(SocketWriterError::Tcp)
    }
}

#[derive(Debug)]
pub enum SocketReaderError {
    TooLong,
    Tcp(embassy_net::tcp::Error),
}

impl From<embassy_net::tcp::Error> for SocketReaderError {
    fn from(e: embassy_net::tcp::Error) -> Self {
        Self::Tcp(e)
    }
}

pub struct SocketReader<'a, 'b: 'a, const CAP: usize> {
    socket: &'a mut TcpSocket<'b>,
    buf: [u8; CAP],
    end: usize,
}

impl<'a, 'b: 'a, const CAP: usize> SocketReader<'a, 'b, CAP> {
    pub fn new(socket: &'a mut TcpSocket<'b>) -> Self {
        Self {
            socket,
            buf: [0; CAP],
            end: 0,
        }
    }

    pub async fn read_all(&mut self) -> Result<(), SocketReaderError> {
        while self.end < self.buf.len() {
            let n =
                embedded_io_async::Read::read(&mut self.socket, &mut self.buf[self.end..]).await?;
            if n == 0 {
                return Ok(());
            } else {
                self.end += n;
            }
        }

        Err(SocketReaderError::TooLong)
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.end]
    }
}

pub async fn get_time_from_influx(
    socket: &mut TcpSocket<'_>,
) -> Result<PrimitiveDateTime, GetTimeError> {
    let mut w = SocketWriter::<1024>::new(socket);

    esp_println::println!("writing to socket");
    write!(w, "GET / HTTP/1.0\r\n").await?;
    write!(w, "Connection: Close\r\n\r\n").await?;
    esp_println::println!("flushing socket");
    w.flush().await?;

    esp_println::println!("reading from socket");
    // I've checked the response; we expect around 500 bytes.
    let mut r = SocketReader::<1024>::new(socket);
    r.read_all().await?;

    esp_println::println!("{}", core::str::from_utf8(&r.buf).unwrap());

    for line in r.buf.split(|&b| b == b'\n') {
        if let Ok(line) = core::str::from_utf8(line) {
            if let Some(date) = parse_header_date(line) {
                return Ok(date);
            }
        }
    }
    Err(GetTimeError::NoTime)
}

#[derive(Debug)]
pub enum SendMeasurementError {
    Write(SocketWriterError),
    Read(SocketReaderError),
    Fmt(core::fmt::Error),
    Tcp(embassy_net::tcp::Error),
}

impl From<SocketReaderError> for SendMeasurementError {
    fn from(v: SocketReaderError) -> Self {
        Self::Read(v)
    }
}

impl From<core::fmt::Error> for SendMeasurementError {
    fn from(v: core::fmt::Error) -> Self {
        Self::Fmt(v)
    }
}

impl From<embassy_net::tcp::Error> for SendMeasurementError {
    fn from(v: embassy_net::tcp::Error) -> Self {
        Self::Tcp(v)
    }
}

impl From<SocketWriterError> for SendMeasurementError {
    fn from(w: SocketWriterError) -> Self {
        Self::Write(w)
    }
}

pub async fn write_measurements(
    socket: &mut TcpSocket<'_>,
    sensor_id: &str,
    now: OffsetDateTime,
    rtc: &Rtc<'_>,
    measurements: impl Iterator<Item = &Measurement>,
) -> Result<(), SendMeasurementError> {
    let mut w = SocketWriter::<1024>::new(socket);

    write!(
        w,
        "POST /api/v2/write?org=treeman-ranch&bucket=temperatures&precision=s HTTP/1.1\r\n"
    )
    .await?;
    write!(w, "Host: {}\r\n", INFLUX_SERVER).await?;
    write!(w, "Authorization: Token {}\r\n", INFLUX_TOKEN).await?;
    write!(w, "Content-type: text/plain; charset=utf-8\r\n").await?;
    write!(w, "Transfer-encoding: chunked\r\n").await?;
    write!(w, "Connection: Close\r\n").await?;
    write!(w, "\r\n").await?;

    let mut body = WriteBuf::<4096>::default();

    let base_secs = now.unix_timestamp();
    let rtc_now = rtc.get_time_ms();
    for Measurement {
        temperature,
        battery,
        rtc_ms,
    } in measurements
    {
        // TODO: is it ok to optimistically write to the body and catch the error? What
        // state does it leave the body in?
        if body.buf.remaining_capacity() <= 100 {
            body.write_http_chunk(&mut w).await;
        }

        let secs_ago = (rtc_now.saturating_sub(*rtc_ms) + 500) / 1000;
        let ts = base_secs - secs_ago as i64;
        let deg = temperature.degrees();
        if let Some(batt) = battery {
            let batt = batt.v();
            writeln!(
                body,
                "tempSensors,sensor_id={sensor_id} temperature={deg:.2},battery={batt:.2} {ts}"
            )?;
        } else {
            writeln!(
                body,
                "tempSensors,sensor_id={sensor_id} temperature={deg:.2} {ts}"
            )?;
        }
        println!("body len {}", body.len());
    }

    body.write_http_chunk(&mut w).await;
    // Write the last (empty) chunk.
    write!(w, "0\r\n\r\n").await?;
    w.flush().await?;

    // We don't expect the server to care if we drop the connection without reading
    // a response (which should have status code 204). But drain the socket just in case.
    // (TODO: maybe a version of read_all that drains and discards, and therefore
    // doesn't need a big buffer?)
    let mut reader = SocketReader::<1024>::new(socket);
    reader.read_all().await?;
    esp_println::println!("{}", core::str::from_utf8(reader.as_bytes()).unwrap());
    socket.close();

    Ok(())
}

#[embassy_executor::task]
pub async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}
