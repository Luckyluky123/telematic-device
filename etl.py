import time
import io
import os
import json
import pandas as pd
import paho.mqtt.client as mqtt
from google.cloud import storage
from google.cloud import bigquery
from datetime import datetime, timezone

# --- CONFIGURATION ---
# GCP Project Details
GCP_PROJECT_ID = "your_project_id"
BQ_DATASET = "your_dataset"
BQ_TABLE = "your_tablet"
GCS_BUCKET_NAME = "your_bucket_name"

CREDENTIALS_PATH = "credentials to key"

# MQTT Configuration
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_USER = "your_user"
MQTT_PASS = "your_password"

# Topics to subscribe to
MQTT_TOPICS = [("esp32/telemetry", 1), ("esp32/gps", 1)]

# Buffer Settings
BUFFER_LIMIT_MSG = 100
BUFFER_LIMIT_TIME = 60

# --- DATA TYPE DEFINITION ---
SCHEMA_MAPPING = {
    'device_id': 'string',      # REQUIRED
    'timestamp': 'datetime',    # REQUIRED
    'rpm': 'Int64',             # NULLABLE INTEGER
    'speed': 'Int64',           # NULLABLE INTEGER
    'fuel_level': 'float64',    # NULLABLE FLOAT    
    'fuel_rate': 'float64',     # NULLABLE FLOAT
    'run_time': 'Int64',        # NULLABLE INTEGER
    'coolant_temp': 'Int64',    # NULLABLE INTEGER
    'dtc_count': 'Int64',       # NULLABLE INTEGER
    'mil_status': 'Int64',      # NULLABLE INTEGER
    'score' : 'float64',
    'lat': 'float64',           # NULLABLE FLOAT
    'lon': 'float64',           # NULLABLE FLOAT
    'ingestion_time': 'datetime',
}

# Global Variables
data_buffer = []
last_upload_time = time.time()

# --- GCP CLIENTS INITIALIZATION ---
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = CREDENTIALS_PATH
try:
    storage_client = storage.Client()
    bq_client = bigquery.Client()
    bucket = storage_client.bucket(GCS_BUCKET_NAME)
except Exception as e:
    print(f"[CRITICAL ERROR] Failed to connect to GCP: {e}")
    exit(1)

def parse_timestamp(col):
    import pandas as pd

    if pd.api.types.is_numeric_dtype(col):
        median_val = col.dropna().median()

        # ms vs s
        if median_val > 1e12:
            dt = pd.to_datetime(col, unit='ms', errors='coerce', utc=True)
        else:
            dt = pd.to_datetime(col, unit='s', errors='coerce', utc=True)
    else:
        dt = pd.to_datetime(col, errors='coerce', utc=True)

    # fallback if anything evaluates to NaT
    dt = dt.fillna(pd.Timestamp.now(tz='UTC'))
    
    dt = dt.dt.floor('us')         
    dt = dt.dt.tz_localize(None)     

    return dt

def enforce_schema(df):
    """
    Aligns the DataFrame with the BigQuery schema.
    Fills missing values, casts data types, and drops redundant columns.
    """
    
    for col in SCHEMA_MAPPING.keys():
        if col not in df.columns:
            df[col] = None  # NULL for missing data

    # Keep only the columns defined in the schema
    df = df[list(SCHEMA_MAPPING.keys())].copy()

    # 3. Timestamp Conversion
    if 'timestamp' in df.columns:
        df['timestamp'] = parse_timestamp(df['timestamp'])
    else:
        df['timestamp'] = parse_timestamp(pd.Series([None] * len(df)))
        
    now_us = pd.Timestamp.now(tz='UTC').floor('us').tz_localize(None)
    df['ingestion_time'] = now_us
    
    # Type Casting
    for col, dtype in SCHEMA_MAPPING.items():
        if col in ('timestamp', 'ingestion_time'):
            continue
        try:
            if dtype == 'Int64':
                df[col] = pd.to_numeric(df[col], errors='coerce').astype('Int64')
            elif dtype == 'float64':
                df[col] = pd.to_numeric(df[col], errors='coerce').astype('float64')
            elif dtype == 'string':
                df[col] = df[col].astype(str)
                df.loc[df[col] == 'nan', col] = "UNKNOWN"
        except Exception as e:
            print(f"[WARN] Problem converting column {col}: {e}")
    
    if 'device_id' in df.columns:
        df = df[df['device_id'].notna() & (df['device_id'] != 'None') & (df['device_id'] != '')]

    return df

def process_and_upload(data):
    """
    Converts the buffer to Parquet format with type enforcement, 
    uploads it to GCS, and triggers a load job into BigQuery.
    """
    global last_upload_time
    if not data: 
        return

    try:
        df_raw = pd.DataFrame(data)
        
        # SCHEMA AND TYPE ENFORCEMENT
        df = enforce_schema(df_raw)

        if df.empty:
            print("[INFO] No valid data to send after filtration.")
            return
        
        parquet_buffer = io.BytesIO()
        
        df.to_parquet(
          parquet_buffer,
          engine='pyarrow',
          index=False,
          allow_truncated_timestamps=True,
          coerce_timestamps='us'   # Forces TIMESTAMP_MICROS
        )
        parquet_buffer.seek(0)

        filename = f"data_{int(time.time())}.parquet"
        blob = bucket.blob(filename)
        blob.upload_from_file(parquet_buffer, content_type='application/octet-stream')
        print(f"[INFO] Uploaded to GCS: {filename} ({len(df)} records)")

        table_ref = f"{GCP_PROJECT_ID}.{BQ_DATASET}.{BQ_TABLE}"
        job_config = bigquery.LoadJobConfig(
            source_format=bigquery.SourceFormat.PARQUET,
            write_disposition=bigquery.WriteDisposition.WRITE_APPEND,
            autodetect=True 
        )
        uri = f"gs://{GCS_BUCKET_NAME}/{filename}"

        load_job = bq_client.load_table_from_uri(uri, table_ref, job_config=job_config)
        load_job.result()  # Wait for the job to complete

        print(f"[INFO] Data loaded into BigQuery: {table_ref}")
        
        # Delete the file after a successful upload
        blob.delete() 
        print(f"[INFO] Temporary file removed from GCS")
        
        last_upload_time = time.time()

    except Exception as e:
        print(f"[ERROR] ETL process failed: {e}")
        

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[INFO] Connected to MQTT broker")
        client.subscribe(MQTT_TOPICS)
    else:
        print(f"[ERROR] MQTT connection failed, code: {rc}")

def on_message(client, userdata, msg):
    print("Message received")
    global data_buffer
    try:
        payload = json.loads(msg.payload.decode())
        
        payload['topic'] = msg.topic 
        data_buffer.append(payload)

        current_time = time.time()
        if len(data_buffer) >= BUFFER_LIMIT_MSG or (current_time - last_upload_time) >= BUFFER_LIMIT_TIME:
            process_and_upload(data_buffer)
            data_buffer.clear()

    except json.JSONDecodeError:
        print(f"[WARNING] Received invalid JSON on topic {msg.topic}")
    except Exception as e:
        print(f"[ERROR] Error processing message: {e}")

# --- MAIN LOOP ---
client = mqtt.Client()
client.username_pw_set(MQTT_USER, MQTT_PASS)
client.on_connect = on_connect
client.on_message = on_message

try:
    print("[INFO] Starting ETL Service (MQTT -> Type Enforced Parquet -> BigQuery)...")
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_forever()
except KeyboardInterrupt:
    if data_buffer:
        print("[INFO] Saving remaining data before exiting...")
        process_and_upload(data_buffer)
    print("[INFO] Service stopped.")
