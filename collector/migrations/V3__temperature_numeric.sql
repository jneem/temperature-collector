BEGIN;
ALTER TABLE temperatures ALTER COLUMN temperature TYPE REAL;
UPDATE temperatures SET temperature=temperature/10;
COMMIT;