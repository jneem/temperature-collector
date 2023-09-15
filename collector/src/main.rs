use std::{net::SocketAddr, sync::Arc, time::SystemTime};

use axum::{extract::State, http::StatusCode, response::IntoResponse, routing::post, Json, Router};
use clap::Parser;
use deadpool_postgres as dp;
use tokio_postgres as pg;
use tracing_subscriber::EnvFilter;

#[derive(Parser)]
struct Args {
    /// The database to connect to, specified tokio-postgres's format.
    database: String,
}

#[derive(Debug, thiserror::Error)]
enum Error {
    #[error("database error: {0}")]
    Database(#[from] pg::Error),
    #[error("database pool error: {0}")]
    Pool(#[from] dp::PoolError),
}

impl IntoResponse for Error {
    fn into_response(self) -> axum::response::Response {
        (StatusCode::INTERNAL_SERVER_ERROR, self.to_string()).into_response()
    }
}

#[derive(Clone)]
struct AppState {
    inner: Arc<dp::Pool>,
}

impl AppState {
    fn new(pool: dp::Pool) -> Self {
        Self {
            inner: Arc::new(pool),
        }
    }
}

#[derive(Clone, Debug, serde::Deserialize)]
pub struct TemperatureSubmission {
    sensor_id: i32,
    temperature: f32,
    battery: Option<i32>,
}

mod embedded {
    use refinery::embed_migrations;
    embed_migrations!("migrations");
}

// TODO: an endpoint for "registering" sensor for the first time, ensuring a unique id and providing a description

#[axum_macros::debug_handler]
async fn collect(
    State(state): State<AppState>,
    Json(submission): Json<TemperatureSubmission>,
) -> Result<(), Error> {
    tracing::info!("got {submission:?}");
    let time = SystemTime::now();

    state
        .inner
        .get()
        .await?
        .query(
            "INSERT INTO temperatures (sensor_id, timestamp, temperature, battery) VALUES ($1, $2, $3, $4);",
            &[&submission.sensor_id, &time, &submission.temperature, &submission.battery],
        )
        .await?;

    Ok(())
}

// TODO: should also have a public key
#[derive(Clone, Debug, serde::Deserialize)]
pub struct SensorRegistration {
    description: String,
}

#[axum_macros::debug_handler]
async fn register(
    State(state): State<AppState>,
    Json(reg): Json<SensorRegistration>,
) -> Result<String, Error> {
    let time = SystemTime::now();

    let sensor_id = state
        .inner
        .get()
        .await?
        .query_one(
            "INSERT INTO sensors (description, updated) VALUES ($1, $2) RETURNING sensor_id;",
            &[&reg.description, &time],
        )
        .await?
        .try_get::<usize, i32>(0)?;

    Ok(sensor_id.to_string())
}

#[tokio::main]
async fn main() {
    let filter = EnvFilter::from_default_env();
    let fmt = tracing_subscriber::fmt::format().pretty();

    tracing_subscriber::fmt()
        .event_format(fmt)
        .with_env_filter(filter)
        .init();

    let args = Args::parse();

    let config: pg::Config = args.database.parse().unwrap();
    let mgr_config = dp::ManagerConfig {
        recycling_method: dp::RecyclingMethod::Fast,
    };
    let mgr = dp::Manager::from_config(config, pg::NoTls, mgr_config);
    let pool = dp::Pool::builder(mgr).max_size(4).build().unwrap();

    {
        let mut client = pool.get().await.unwrap();
        let client: &mut pg::Client = &mut client;
        embedded::migrations::runner().get_migrations();
        embedded::migrations::runner()
            .run_async(client)
            .await
            .unwrap();
    }

    let app = Router::new()
        .route("/collect", post(collect))
        .route("/register", post(register))
        .with_state(AppState::new(pool));
    let addr = SocketAddr::from(([0, 0, 0, 0], 3000));
    axum::Server::bind(&addr)
        .serve(app.into_make_service())
        .await
        .unwrap();
}
