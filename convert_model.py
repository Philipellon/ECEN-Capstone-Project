import tensorflow as tf

# Load your Keras model (adjust the filename as needed)
model = tf.keras.models.load_model('command_model_cnn.keras')

# Create a converter object from your model
converter = tf.lite.TFLiteConverter.from_keras_model(model)

# Convert the model
tflite_model = converter.convert()

# Save the converted model to disk
with open("model.tflite", "wb") as f:
    f.write(tflite_model)
print("Model converted to TensorFlow Lite and saved as model.tflite.")
