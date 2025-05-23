import model 
import sys

def main(argc, arc):

    args = sys.argv[1:]

    if len(args) != 1:
        print("Usage: python train.py <data_dir>")
        sys.exit(1)
    data_dir = args[0]

    appraoch = "regression"  # or "classification"

    img2compaction = model.Image2Compaction(data_dir, approach=appraoch) 
    img2compaction.load_data()
    img2compaction.build_model(epochs=30)
    img2compaction.plot_training_results()
    img2compaction.save_model("model.keras")

if __name__ == '__main__':
    main(sys.argv, len(sys.argv))