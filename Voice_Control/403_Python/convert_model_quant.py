import tensorflow as tf

# Load your trained model
model = tf.keras.models.load_model('command_model_cnn.keras')

# Create the converter from the Keras model
converter = tf.lite.TFLiteConverter.from_keras_model(model)

# Set the converter to use optimization (quantization)
converter.optimizations = [tf.lite.Optimize.DEFAULT]

# (Optional) Define a representative dataset for better quantization calibration
def representative_data_gen():
    # Replace this with a loop over a small set of sample inputs from your training data
    # For example, if your model input shape is (batch_size, 13, 32, 1):
    import numpy as np
    for _ in range(100):
        # Generate a random sample with the correct input shape (adjust as needed)
        sample_input = np.random.rand(1, 13, 32, 1).astype('float32')
        yield [sample_input]

converter.representative_dataset = representative_data_gen

# Convert the model to a quantized TFLite model
tflite_quant_model = converter.convert()

# Save the quantized model to disk
with open("model_quant.tflite", "wb") as f:
    f.write(tflite_quant_model)

print("Quantized model saved as model_quant.tflite.")

