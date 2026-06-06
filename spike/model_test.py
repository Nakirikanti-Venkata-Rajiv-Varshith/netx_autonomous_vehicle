import pandas as pd
import joblib

model = joblib.load("routing_xgb.pkl")

sample = pd.DataFrame([{
    "bandwidth_mbps": upload_speed,
    "cpu_percent": cpu_usage,
    "gpu_percent": gpu_usage,
    "ram_percent": ram_usage,
    "rtt_ms": self.rtt_ms if self.rtt_ms is not None else 0,
    "jitter_ms": self.jitter_ms,
    "motion_score": profile["motion_score"],
    "change_score": profile["change_score"],
}])

cloud_score = float(
    model.predict_proba(sample)[0][1]
)

edge_score = 1.0 - cloud_score

route = (
    "offboard"
    if cloud_score > 0.5
    else "onboard"
)

print(
    f"cloud={cloud_score:.4f} "
    f"edge={edge_score:.4f} "
    f"route={route}"
)