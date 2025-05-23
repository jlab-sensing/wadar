import model 
import sys
import pandas as pd
import pathlib
import json
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt

def main(argc, arc):

    args = sys.argv[1:]
    if len(args) != 1:
        print("Usage: python test.py <data_dir>")
        sys.exit(1)
    data_dir = args[0]

    # Run the model
    model_path = "model.keras"
    img2compaction = model.Image2Compaction(data_dir, approach="regression")
    img2compaction.load_model(model_path)

    df = pd.read_csv(pathlib.Path(data_dir) / "dataset.csv")
    image_files = df["filename"].tolist()
    labels = df["label"].tolist()
    predictions = []

    for i in range(len(image_files)):
        image = model.load_image(image_files[i], img_height=160, img_width=160)
        image = tf.expand_dims(image, axis=0)
        prediction = img2compaction.model.predict(image)
        print(f"Image: {image_files[i]}, Prediction: {prediction[0][0]:.2f}, Actual: {labels[i]:.2f}")
        predictions.append(prediction[0][0])
    
    goodActual = []
    goodPred = []
    goodCorrect = 0
    goodIncorrect = 0
    nonIdealActual = []
    nonIdealPred = []
    nonIdealCorrect = 0
    nonIdealIncorrect = 0
    unviableActual = []
    unviablePred = []
    unviableCorrect = 0
    unviableIncorrect = 0
    for i in range(len(image_files)):
        if labels[i] <= 1.4:
            goodActual.append(labels[i])
            goodPred.append(predictions[i])
            if predictions[i] <= 1.4:
                goodCorrect += 1
            else:
                goodIncorrect += 1
        elif labels[i] <= 1.8:
            nonIdealActual.append(labels[i])
            nonIdealPred.append(predictions[i])
            if predictions[i] <= 1.8:
                nonIdealCorrect += 1
            else:
                nonIdealIncorrect += 1
        elif labels[i] < 2.0:
            unviableActual.append(labels[i])
            unviablePred.append(predictions[i])
            if predictions[i] < 2.0:
                unviableCorrect += 1
            else:
                unviableIncorrect += 1

    # print(f"Good: {goodCorrect} correct, {goodIncorrect} incorrect")
    # print(f"Non-Ideal: {nonIdealCorrect} correct, {nonIdealIncorrect} incorrect")
    # print(f"Unviable: {unviableCorrect} correct, {unviableIncorrect} incorrect")
    # print(f"Good: {goodCorrect/(goodCorrect+goodIncorrect):.2f} correct")
    # print(f"Non-Ideal: {nonIdealCorrect/(nonIdealCorrect+nonIdealIncorrect):.2f} correct")
    # print(f"Unviable: {unviableCorrect/(unviableCorrect+unviableIncorrect):.2f} correct")

    # Confusion matrix assembly (only for good and non-ideal)
    totalGood = goodCorrect + goodIncorrect
    totalNonIdeal = nonIdealCorrect + nonIdealIncorrect
    mtx = np.array([[goodCorrect, goodIncorrect],
                    [nonIdealIncorrect, nonIdealCorrect]])
    mtx = mtx.astype('float') / mtx.sum(axis=1)[:, np.newaxis]  # Normalize by row (i.e. by the number of samples in each class)
    print(mtx)

    goodActual = np.array(goodActual)
    goodPred = np.array(goodPred)
    nonIdealActual = np.array(nonIdealActual)
    nonIdealPred = np.array(nonIdealPred)
    unviableActual = np.array(unviableActual)
    unviablePred = np.array(unviablePred)

    # Confusion matrix plot (CoPilot go brr...)
    plt.figure(figsize=(8, 8))
    plt.subplot(1, 2, 1)
    plt.imshow(mtx, interpolation='nearest', cmap=plt.cm.Blues)
    plt.title('Confusion matrix')
    plt.colorbar()
    tick_marks = np.arange(len(["Good", "Non-Ideal"]))
    plt.xticks(tick_marks, ["Good", "Non-Ideal"], rotation=45)
    plt.yticks(tick_marks, ["Good", "Non-Ideal"])
    plt.tight_layout()
    plt.ylabel('True label')    


    # Box and whisker plots for each class (CoPilot again)
    plt.figure(figsize=(10, 6))
    plt.subplot(1, 3, 1)
    plt.boxplot(goodActual, positions=[1], widths=0.5)
    plt.boxplot(goodPred, positions=[2], widths=0.5)
    plt.title("Good")
    plt.xticks([1, 2], ["Actual", "Predicted"])
    plt.ylabel("Bulk Density (g/cm^3)")
    plt.ylim([0, 2.5])
    plt.subplot(1, 3, 2)
    plt.boxplot(nonIdealActual, positions=[1], widths=0.5)
    plt.boxplot(nonIdealPred, positions=[2], widths=0.5)
    plt.title("Non-Ideal")
    plt.xticks([1, 2], ["Actual", "Predicted"])
    plt.ylabel("Bulk Density (g/cm^3)")
    plt.ylim([0, 2.5])
    plt.subplot(1, 3, 3)
    plt.boxplot(unviableActual, positions=[1], widths=0.5)
    plt.boxplot(unviablePred, positions=[2], widths=0.5)
    plt.title("Unviable")
    plt.xticks([1, 2], ["Actual", "Predicted"])
    plt.ylabel("Bulk Density (g/cm^3)")
    plt.ylim([0, 2.5])
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main(sys.argv, len(sys.argv))