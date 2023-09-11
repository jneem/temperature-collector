CREATE TABLE sensors (
    sensor_id SERIAL PRIMARY KEY,
    description VARCHAR(128),
    updated TIMESTAMP
);

ALTER TABLE temperatures
ADD CONSTRAINT fk_sensor_id FOREIGN KEY (sensor_id) REFERENCES sensors(sensor_id);