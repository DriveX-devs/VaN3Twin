import xgboost as xgb
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.metrics import ConfusionMatrixDisplay, classification_report, accuracy_score
from sklearn.impute import SimpleImputer

THRESHOLD = 0.5

df = pd.read_csv("coordinations_seed42.csv", index_col=False)
# df = df[(df["execution_success"] == 0) | (df["execution_success"] == 2)]
df.loc[df["execution_success"] == 1, "execution_success"] = 0
df.loc[df["execution_success"] == 2, "execution_success"] = 1

df = df.drop(["coordination_id", "sim_time_ms"], axis=1)
df["std_speed_ahead"] = df["std_speed_ahead"].fillna(0)
df["std_speed_behind"] = df["std_speed_behind"].fillna(0)

X = df.drop("execution_success", axis=1) 
y = df["execution_success"]

si = SimpleImputer(missing_values=-2000, fill_value=np.nan)
si.set_output(transform="pandas")
X = si.fit_transform(X)

print(X.shape)

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, stratify=y, random_state=42)

model = xgb.XGBClassifier(
    n_estimators=200,
    learning_rate=0.05,
    max_depth=3,            
    reg_lambda=5,
    reg_alpha=5,
    colsample_bytree=0.7,
    subsample=0.7,
    eval_metric='logloss'
)

model.fit(X_train, y_train)

y_pred = model.predict(X_train)
y_pred_prob = model.predict_proba(X_train)
y_pred = (y_pred_prob >= THRESHOLD).astype(int)[:, 1]
print("Accuracy on Training Set:", accuracy_score(y_train, y_pred))
# print("Classification Report on Training Set:\n", classification_report(y_train, y_pred))

y_pred = model.predict(X_test)
y_pred_prob = model.predict_proba(X_test)
y_pred = (y_pred_prob >= THRESHOLD).astype(int)[:, 1]
print("Accuracy on Training Set:", accuracy_score(y_test, y_pred))
ConfusionMatrixDisplay.from_predictions(y_test, y_pred)
plt.title(f"Confusion Matrix (th = 0.5) - XGBoost")
plt.show()
plt.close()

importances_dict = model.get_booster().get_score(importance_type='gain')
# Convert to list of tuples and sort
importances_sorted = sorted(importances_dict.items(), key=lambda x: x[1])
# Split names and scores
sorted_features = [item[0] for item in importances_sorted]
sorted_importances = [item[1] for item in importances_sorted]

# Plot

plt.barh(sorted_features, sorted_importances)
plt.title('Feature Importance (XGBoost)')
plt.xlabel('Importance (by gain)')
plt.ylabel('Feature')
plt.tight_layout()
plt.show()
plt.close()