# Script that loads those feature arrays, trains a classifier in TensorFlow/PyTorch, and saves the trained model 
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
import tensorflow as tf
from tensorflow.keras import layers, models
from sklearn.metrics import confusion_matrix, classification_report
import matplotlib.pyplot as plt
import pickle
from sklearn.utils.class_weight import compute_class_weight

# ========== 1. Load the Preprocessed Features and Labels ==========
X = np.load("features.npy")
y = np.load("labels.npy")

# Encode the string labels as integers
label_encoder = LabelEncoder()
y_encoded = label_encoder.fit_transform(y)
num_classes = len(label_encoder.classes_)

# ========== 2. Train/Test Split ==========
X_train, X_test, y_train, y_test = train_test_split(
    X, y_encoded, test_size=0.2, random_state=42, stratify=y_encoded
)

print("Training features shape:", X_train.shape)
print("Training labels shape:", y_train.shape)
print("Test features shape:", X_test.shape)
print("Test labels shape:", y_test.shape)

# ========== 3. Define the Model ==========
input_dim = X_train.shape[1]  # e.g., 13 MFCC coefficients
model = models.Sequential([
    layers.Dense(64, activation='relu', input_shape=(input_dim,)),
    layers.Dense(32, activation='relu'),
    layers.Dense(num_classes, activation='softmax')
])

model.compile(
    optimizer='adam',
    loss='sparse_categorical_crossentropy',
    metrics=['accuracy']
)

# ========== 4. Compute Class Weights ==========
class_weights = compute_class_weight('balanced', classes=np.unique(y_encoded), y=y_encoded)
class_weight_dict = {i: weight for i, weight in enumerate(class_weights)}
print("Class weights:", class_weight_dict)

# ========== 5. Train the Model with Class Weights ==========
history = model.fit(
    X_train, 
    y_train, 
    epochs=20, 
    batch_size=32, 
    validation_split=0.1,
    class_weight=class_weight_dict  # <-- Applying class weights here
)

# ========== 6. Evaluate the Model on the Test Set ==========
test_loss, test_acc = model.evaluate(X_test, y_test)
print("Test accuracy:", test_acc)

# ========== 7. Save the Model in the Recommended Keras Format ==========
model.save("command_model.keras")
print("Trained model saved as 'command_model.keras'.")

# Save the label encoder for future reference
with open("label_encoder.pkl", "wb") as f:
    pickle.dump(label_encoder, f)
print("Label encoder saved as 'label_encoder.pkl'.")

# ========== 8. Plot Training History (Loss & Accuracy) ==========
plt.figure(figsize=(12, 4))

# Plot Loss
plt.subplot(1, 2, 1)
plt.plot(history.history['loss'], label='Training Loss')
plt.plot(history.history['val_loss'], label='Validation Loss')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.title('Training and Validation Loss')
plt.legend()

# Plot Accuracy
plt.subplot(1, 2, 2)
plt.plot(history.history['accuracy'], label='Training Accuracy')
plt.plot(history.history['val_accuracy'], label='Validation Accuracy')
plt.xlabel('Epoch')
plt.ylabel('Accuracy')
plt.title('Training and Validation Accuracy')
plt.legend()

plt.tight_layout()
plt.show()

# ========== 9. (Optional) Confusion Matrix and Classification Report ==========
y_pred_proba = model.predict(X_test)
y_pred = np.argmax(y_pred_proba, axis=1)

cm = confusion_matrix(y_test, y_pred)
print("\nConfusion Matrix:")
print(cm)

print("\nClassification Report:")
class_names = label_encoder.classes_
print(classification_report(y_test, y_pred, target_names=class_names))
