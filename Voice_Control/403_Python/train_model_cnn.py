import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from sklearn.utils.class_weight import compute_class_weight
from sklearn.metrics import confusion_matrix, classification_report
import tensorflow as tf
from tensorflow.keras import layers, models
import matplotlib.pyplot as plt
import pickle

# ========= ASSUMPTION =========
# This script expects that you have preprocessed your audio files into full 2D MFCC (or spectrogram) features.
# The file "full_features.npy" should have shape: (num_samples, n_mfcc, time_steps)
# For example, if you use 13 MFCC coefficients and pad/truncate each sample to 32 time frames, then:
#    X.shape = (num_samples, 13, 32)
# We then add a channel dimension to use a CNN, so the final input shape is (13, 32, 1).
# ==============================

# ========== 1. Load the Full Spectrogram Features and Labels ==========
X = np.load("full_features.npy")  # shape: (num_samples, n_mfcc, time_steps)
y = np.load("labels.npy")           # labels remain the same

# Add a channel dimension for CNN input: new shape becomes (num_samples, n_mfcc, time_steps, 1)
X = X[..., np.newaxis]

# Encode the string labels as integers
label_encoder = LabelEncoder()
y_encoded = label_encoder.fit_transform(y)
num_classes = len(label_encoder.classes_)

# ========== 2. Train/Test Split ==========
X_train, X_test, y_train, y_test = train_test_split(
    X, y_encoded, test_size=0.2, random_state=42, stratify=y_encoded
)
print("Training features shape:", X_train.shape)
print("Test features shape:", X_test.shape)

# ========== 3. Compute Class Weights ==========
class_weights = compute_class_weight('balanced', classes=np.unique(y_encoded), y=y_encoded)
class_weight_dict = {i: weight for i, weight in enumerate(class_weights)}
print("Class weights:", class_weight_dict)

# ========== 4. Define a CNN Model ==========
input_shape = X_train.shape[1:]  # e.g., (13, 32, 1)
model = models.Sequential([
    layers.Conv2D(32, kernel_size=(3, 3), activation='relu', input_shape=input_shape),
    layers.MaxPooling2D(pool_size=(2, 2)),
    layers.Conv2D(64, kernel_size=(3, 3), activation='relu'),
    layers.MaxPooling2D(pool_size=(2, 2)),
    layers.Flatten(),
    layers.Dense(128, activation='relu'),
    layers.Dense(num_classes, activation='softmax')
])

model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

# ========== 5. Train the CNN Model ==========
history = model.fit(
    X_train, 
    y_train, 
    epochs=30,         # Try more epochs as CNNs may require longer training
    batch_size=32, 
    validation_split=0.1,
    class_weight=class_weight_dict
)

# ========== 6. Evaluate the Model ==========
test_loss, test_acc = model.evaluate(X_test, y_test)
print("Test accuracy:", test_acc)

# ========== 7. Save the Model and Label Encoder ==========
model.save("command_model_cnn.keras")
print("Trained model saved as 'command_model_cnn.keras'.")

with open("label_encoder.pkl", "wb") as f:
    pickle.dump(label_encoder, f)
print("Label encoder saved as 'label_encoder.pkl'.")

# ========== 8. Plot Training History ==========
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

# ========== 9. Confusion Matrix and Classification Report ==========
y_pred_proba = model.predict(X_test)
y_pred = np.argmax(y_pred_proba, axis=1)
cm = confusion_matrix(y_test, y_pred)
print("Confusion Matrix:")
print(cm)

print("\nClassification Report:")
print(classification_report(y_test, y_pred, target_names=label_encoder.classes_))
