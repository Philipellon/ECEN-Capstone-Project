import os
import librosa
import numpy as np

# ====== Parameters ======
SAMPLE_RATE = 16000      # Standard sampling rate for speech
N_MFCC = 13              # Number of MFCC coefficients
HOP_LENGTH = 512         # Determines the time resolution
FIXED_LENGTH = 32        # Desired number of time frames per sample

def extract_full_mfcc(file_path, sample_rate=SAMPLE_RATE, n_mfcc=N_MFCC, hop_length=HOP_LENGTH, fixed_length=FIXED_LENGTH):
    """
    Load an audio file and compute its MFCCs.
    Pads or truncates the resulting MFCC matrix along the time axis
    so that each sample has exactly 'fixed_length' time frames.
    """
    try:
        # Load the audio file
        signal, sr = librosa.load(file_path, sr=sample_rate)
        # Compute MFCC features; shape will be (n_mfcc, t)
        mfcc = librosa.feature.mfcc(y=signal, sr=sr, n_mfcc=n_mfcc, hop_length=hop_length)
        # Get the current number of time frames
        t = mfcc.shape[1]
        if t < fixed_length:
            # Pad with zeros on the right if too short
            pad_width = fixed_length - t
            mfcc = np.pad(mfcc, pad_width=((0, 0), (0, pad_width)), mode='constant')
        elif t > fixed_length:
            # Truncate to fixed_length if too long
            mfcc = mfcc[:, :fixed_length]
        return mfcc
    except Exception as e:
        print(f"Error processing file {file_path}: {e}")
        return None

def preprocess_full_features():
    """
    Walks through each subfolder in the current directory (each representing a command label),
    extracts the full 2D MFCC features from every WAV file (with fixed time dimension),
    and saves the resulting features and labels to .npy files.
    """
    features = []  # List to store MFCC arrays
    labels = []    # Corresponding labels

    # Get current working directory (assumes your command folders are in the same directory)
    base_dir = os.getcwd()
    
    # Iterate through each item in the base directory
    for folder in os.listdir(base_dir):
        folder_path = os.path.join(base_dir, folder)
        # Process folders that are not hidden
        if os.path.isdir(folder_path) and not folder.startswith('.'):
            print(f"Processing label: {folder}")
            # Each folder is considered a label; process all .wav files in it
            for file_name in os.listdir(folder_path):
                if file_name.lower().endswith(".wav"):
                    file_path = os.path.join(folder_path, file_name)
                    mfcc = extract_full_mfcc(file_path)
                    if mfcc is not None:
                        features.append(mfcc)
                        labels.append(folder)
    
    # Convert lists to NumPy arrays
    features = np.array(features)  # Expected shape: (num_samples, n_mfcc, fixed_length)
    labels = np.array(labels)
    
    print("Full features shape:", features.shape)
    print("Labels shape:", labels.shape)
    
    # Save arrays to disk
    np.save("full_features.npy", features)
    np.save("labels.npy", labels)
    print("Saved 'full_features.npy' and 'labels.npy'.")

if __name__ == "__main__":
    preprocess_full_features()
