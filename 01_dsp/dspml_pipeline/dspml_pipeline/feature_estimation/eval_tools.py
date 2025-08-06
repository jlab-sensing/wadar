from dspml_pipeline.feature_estimation.ridge_regression import RidgeRegression
from dspml_pipeline.feature_estimation.random_forest import RandomForest
from dspml_pipeline.feature_estimation.xgboost_tree import XGBoostTree
from dspml_pipeline.feature_estimation.svr import SVRRegression
from dspml_pipeline.results import update_results
from dspml_pipeline.feature_estimation.mlp import MLPRegression

def evaluate_classic_models(target_dir, feature_array, labels, tune_model_params, feature_name):
    
    ridgeRegressor = RidgeRegression()
    models, metrics = ridgeRegressor.full_monty(feature_array, labels)
    update_results(target_dir, feature_name, f"Ridge Regression Degree", metrics)

    # ==

    randomForest = RandomForest(tune_model_params=tune_model_params)
    model, metrics = randomForest.full_monty(feature_array, labels)
    update_results(target_dir, feature_name, "Random Forest", metrics)

    # ==

    gbTree = XGBoostTree(tune_model_params)
    model, metrics = gbTree.full_monty(feature_array, labels)
    update_results(target_dir, feature_name, "Gradient Boosted Tree", metrics)

    # ==

    svr = SVRRegression(tune_model_params)
    model, metrics = svr.full_monty(feature_array, labels)
    update_results(target_dir, feature_name, "SVR", metrics)

def evaluate_deep_models(target_dir, feature_array, labels, feature_name):
    mlp = MLPRegression()
    _, metrics = mlp.full_monty(feature_array, labels)
    update_results(target_dir, feature_name, "Multi Layer Pecepetron", metrics)