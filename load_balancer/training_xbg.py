import pandas as pd
import numpy as np
import joblib
import xgboost
import sklearn
from sklearn.model_selection import train_test_split
from sklearn.metrics import (
    classification_report,
    confusion_matrix,
    roc_auc_score
)

from xgboost import XGBClassifier

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
# CLASS IMBALANCE
# =====================================================

num_negative = (y_train == 0).sum()
num_positive = (y_train == 1).sum()

scale_pos_weight = num_negative / num_positive

print("scale_pos_weight =", scale_pos_weight)

# =====================================================
# XGBOOST
# =====================================================

model = XGBClassifier(
    objective="binary:logistic",
    n_estimators=300,
    max_depth=5,
    learning_rate=0.05,
    subsample=0.8,
    colsample_bytree=0.8,
    scale_pos_weight=scale_pos_weight,
    eval_metric="logloss",
    random_state=42,
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

joblib.dump(model, "routing_xgb.pkl")
model.save_model("routing_xgb.json")

print("\nSaved -> routing_xgb.pkl")
print("Saved -> routing_xgb.json")


print("\nEnvironment")
print("XGBoost :", xgboost.__version__)
print("Sklearn :", sklearn.__version__)
print("NumPy   :", np.__version__)

print("\nFeature Order")
print(features)