ALTER TABLE temperatures ADD COLUMN battery INT;
ALTER TABLE temperatures ALTER COLUMN temperature SET NOT NULL;
ALTER TABLE temperatures ALTER COLUMN sensor_id SET NOT NULL;
