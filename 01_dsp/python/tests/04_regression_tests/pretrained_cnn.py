import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
import pandas as pd
from PIL import Image
import tensorflow as tf
from _06_hermes.parameters import num2label
from tensorflow.data import Dataset

if __name__ == "__main__":

    dataset_dir = "../../data/combined-soil-compaction-dataset"
    feature_file_name = "features.csv"

    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X = np.abs(hydros.X)
    y = hydros.y

    print("X shape:", X.shape)

    output_dir = "../../data/combined-soil-compaction-dataset/images"
    os.makedirs(output_dir, exist_ok=True)

    labels = []

    for idx, sample in enumerate(X):
        
        sample_min = sample.min()           # normalize to 0-255 
        sample_max = sample.max()
        normalized_sample = 255 * (sample - sample_min) / (sample_max - sample_min + 1e-8)

        img = Image.fromarray(normalized_sample.astype(np.uint8))

        label = y[idx]


        file_name =  f"sample_{idx:03d}.png"
        img.save(os.path.join(output_dir, file_name))

        labels.append({
            'filename': file_name,
            'label': label
        })

    print(f"Saved {len(X)} images to {output_dir}")
    df = pd.DataFrame(labels)
    df.to_csv(output_dir + "/labels.csv", index=False)


    # https://www.tensorflow.org/tutorials/images/transfer_learning

    # ====================================================
    # Data preprocessing
    # ====================================================

    # Data download

    train_dir = output_dir
    validation_dir = output_dir

    BATCH_SIZE = 32
    IMG_SIZE = (160, 160)

    df = pd.read_csv(output_dir + "/labels.csv")
    class_names = df['label'].values

    class_labels = [num2label(label) for label in class_names]

    def load_image(filename):
        img_path = os.path.join(output_dir, filename)
        img = Image.open(img_path).convert('RGB')
        img = img.resize(IMG_SIZE)
        img_array = np.array(img)
        return img_array

    images = []
    labels_numeric = []

    for idx, row in df.iterrows():
        img_array = load_image(row['filename'])
        images.append(img_array)
        labels_numeric.append(int(df.loc[idx, 'label']))  # or float if regression


    images = np.stack(images)
    labels_numeric = np.array(labels_numeric)

    # Split into train/validation (e.g., 80/20 split)
    split_idx = int(0.8 * len(images))
    train_images, val_images = images[:split_idx], images[split_idx:]
    train_labels, val_labels = labels_numeric[:split_idx], labels_numeric[split_idx:]

    train_dataset = Dataset.from_tensor_slices((train_images, train_labels)).batch(BATCH_SIZE)
    validation_dataset = Dataset.from_tensor_slices((val_images, val_labels)).batch(BATCH_SIZE)
    
    # plt.figure(figsize=(10, 10))
    # for images, labels in train_dataset.take(1):
    #     for i in range(9):
    #         ax = plt.subplot(3, 3, i + 1)
    #         plt.imshow(images[i].numpy().astype("uint8"))
    #         plt.title(f"This is {labels[i]}")
    #         plt.axis("off")
    # plt.show()

    val_batches = tf.data.experimental.cardinality(validation_dataset)
    test_dataset = validation_dataset.take(val_batches // 5)
    validation_dataset = validation_dataset.skip(val_batches // 5)

    print('Number of validation batches: %d' % tf.data.experimental.cardinality(validation_dataset))
    print('Number of test batches: %d' % tf.data.experimental.cardinality(test_dataset))

    # Configure the dataset for performance

    AUTOTUNE = tf.data.AUTOTUNE

    train_dataset = train_dataset.prefetch(buffer_size=AUTOTUNE)
    validation_dataset = validation_dataset.prefetch(buffer_size=AUTOTUNE)
    test_dataset = test_dataset.prefetch(buffer_size=AUTOTUNE)

    # No data augmentation because radargrams are slow time x fast time, can't be flipped or 
    # rotated like images.

    # Rescale pixel values 

    rescale = tf.keras.layers.Rescaling(1./127.5, offset=-1) # Rescale from [0, 255] to [-1, 1]

    # ====================================================
    # Create the base model from the pre-trained convnets
    # ====================================================

    # Create the base model from the pre-trained model MobileNet V2
    IMG_SHAPE = IMG_SIZE + (3,)
    base_model = tf.keras.applications.MobileNetV2(input_shape=IMG_SHAPE,
                                                include_top=False,
                                                weights='imagenet')
    
    image_batch, label_batch = next(iter(train_dataset))
    feature_batch = base_model(image_batch)
    print(feature_batch.shape)

    # =====================================================
    # Feature extraction
    # =====================================================

    base_model.trainable = False

    # Let's take a look at the base model architecture
    base_model.summary()

    # Add a classification head

    global_average_layer = tf.keras.layers.GlobalAveragePooling2D()
    feature_batch_average = global_average_layer(feature_batch)
    print(feature_batch_average.shape)

    prediction_layer = tf.keras.layers.Dense(1, activation='sigmoid')
    prediction_batch = prediction_layer(feature_batch_average)
    print(prediction_batch.shape)

    inputs = tf.keras.Input(shape=(160, 160, 3))
    x = rescale(inputs)
    x = base_model(x, training=False)
    x = global_average_layer(x)
    x = tf.keras.layers.Dropout(0.2)(x)
    outputs = prediction_layer(x)
    model = tf.keras.Model(inputs, outputs)

    model.summary()

    len(model.trainable_variables)

    # tf.keras.utils.plot_model(model, show_shapes=True)    # don't have graphviz installed

    base_learning_rate = 0.0001
    model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=base_learning_rate),
                loss=tf.keras.losses.BinaryCrossentropy(),
                metrics=[tf.keras.metrics.BinaryAccuracy(threshold=0.5, name='accuracy')])
    
    initial_epochs = 10

    loss0, accuracy0 = model.evaluate(validation_dataset)

    history = model.fit(train_dataset,
                        epochs=initial_epochs,
                        validation_data=validation_dataset)
    
    acc = history.history['accuracy']
    val_acc = history.history['val_accuracy']

    loss = history.history['loss']
    val_loss = history.history['val_loss']

    plt.figure(figsize=(8, 8))
    plt.subplot(2, 1, 1)
    plt.plot(acc, label='Training Accuracy')
    plt.plot(val_acc, label='Validation Accuracy')
    plt.legend(loc='lower right')
    plt.ylabel('Accuracy')
    plt.ylim([min(plt.ylim()),1])
    plt.title('Training and Validation Accuracy')

    plt.subplot(2, 1, 2)
    plt.plot(loss, label='Training Loss')
    plt.plot(val_loss, label='Validation Loss')
    plt.legend(loc='upper right')
    plt.ylabel('Cross Entropy')
    plt.ylim([0,1.0])
    plt.title('Training and Validation Loss')
    plt.xlabel('epoch')
    plt.show()