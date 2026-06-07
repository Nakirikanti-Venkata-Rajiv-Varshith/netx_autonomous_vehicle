import pandas as pd
import numpy as np
import joblib

from sklearn.model_selection import train_test_split
from sklearn.metrics import (
    classification_report,
    confusion_matrix,
    roc_auc_score
)

from sklearn.ensemble import RandomForestClassifier

# =====================================================
# LOAD DATA
# =====================================================

df = pd.read_csv("routing_ground_truth.csv")

# =====================================================
# FEATURES
# =====================================================

features = [
    "bandwidth_mbps",
    "cpu_percent",
    "gpu_percent",
    "ram_percent",
    "rtt_ms",
    "jitter_ms",
    "motion_score",
    "change_score",
]

X = df[features].copy()
y = df["best_route"]

# =====================================================
# CLEAN
# =====================================================

X = X.replace([np.inf, -np.inf], np.nan)

for col in X.columns:
    X[col] = X[col].fillna(X[col].median())

# =====================================================
# SPLIT
# =====================================================

X_train, X_test, y_train, y_test = train_test_split(
    X,
    y,
    test_size=0.20,
    stratify=y,
    random_state=42,
)

# =====================================================
# RANDOM FOREST
# =====================================================

model = RandomForestClassifier(
    n_estimators=500,
    max_depth=10,
    min_samples_leaf=5,
    class_weight="balanced",
    random_state=42,
    n_jobs=-1,
)

# =====================================================
# TRAIN
# =====================================================

model.fit(X_train, y_train)

# =====================================================
# EVALUATE
# =====================================================

pred = model.predict(X_test)

pred_prob = model.predict_proba(X_test)[:, 1]

print("\nClassification Report\n")
print(classification_report(y_test, pred))

print("\nConfusion Matrix\n")
print(confusion_matrix(y_test, pred))

print("\nROC AUC\n")
print(roc_auc_score(y_test, pred_prob))

# =====================================================
# FEATURE IMPORTANCE
# =====================================================

importance = pd.DataFrame({
    "feature": features,
    "importance": model.feature_importances_
})

importance = importance.sort_values(
    "importance",
    ascending=False
)

print("\nFeature Importance\n")
print(importance)

# =====================================================
# SAVE
# =====================================================

joblib.dump(model, "routing_rf.pkl")
joblib.dump(features, "routing_rf_features.pkl")

print("\nSaved -> routing_rf.pkl")
print("Saved -> routing_rf_features.pkl")

print("\nFeature Order")
print(features)