from fastapi import FastAPI
import random

app = FastAPI()

@app.get("/")
def home():
    return {"status": "running"}

@app.post("/predict")
def predict(data: dict):
    return {
        "prediction": round(random.random(), 3)
    }
