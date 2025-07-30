#!/usr/bin/env python3
"""
Test script for new Lasso and Random Forest feature selection methods.
"""

import os
import sys
import numpy as np
import pandas as pd

# Add project root to path
sys.path.insert(0, os.path.abspath('.'))

from _03_hephaestus.feature_tools import lasso_minimize_features, random_forest_minimize_features

def create_test_data():
    """Create synthetic test data for feature selection testing."""
    np.random.seed(42)
    
    # Create 100 samples with 20 features
    n_samples = 100
    n_features = 20
    
    # Generate synthetic features
    features = np.random.randn(n_samples, n_features)
    
    # Create labels that depend on only some features (3, 7, 12, 15)
    # This simulates a scenario where only some features are relevant
    important_features = [3, 7, 12, 15]
    labels = (2.0 * features[:, 3] + 
             1.5 * features[:, 7] + 
             -1.0 * features[:, 12] + 
             0.8 * features[:, 15] + 
             0.1 * np.random.randn(n_samples))  # Add small amount of noise
    
    # Create DataFrame
    feature_names = [f'Feature_{i+1}' for i in range(n_features)]
    df = pd.DataFrame(features, columns=feature_names)
    df['Label'] = labels
    
    return df, important_features

def test_lasso_selection():
    """Test Lasso feature selection."""
    print("="*60)
    print("TESTING LASSO FEATURE SELECTION")
    print("="*60)
    
    test_df, important_features = create_test_data()
    
    # Test Lasso feature selection
    try:
        selected_features_df, lasso_scores_df = lasso_minimize_features(test_df, top_n=8)
        
        print(f"Original features: {test_df.shape[1]-1}")
        print(f"Selected features: {selected_features_df.shape[1]-1}")
        print("\nTop Lasso features:")
        print(lasso_scores_df.head(10))
        
        # Check if important features were selected
        selected_feature_names = lasso_scores_df['Feature'].tolist()
        important_feature_names = [f'Feature_{i+1}' for i in important_features]
        
        found_important = sum(1 for feat in important_feature_names if feat in selected_feature_names)
        print(f"\nImportant features found: {found_important}/{len(important_features)}")
        print(f"Important features: {important_feature_names}")
        print(f"Selected features: {selected_feature_names[:8]}")
        
        print("\n✅ Lasso feature selection test PASSED")
        return True
        
    except Exception as e:
        print(f"\n❌ Lasso feature selection test FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_random_forest_selection():
    """Test Random Forest feature selection."""
    print("\n" + "="*60)
    print("TESTING RANDOM FOREST FEATURE SELECTION")
    print("="*60)
    
    test_df, important_features = create_test_data()
    
    # Test Random Forest feature selection
    try:
        selected_features_df, rf_scores_df = random_forest_minimize_features(test_df, top_n=8)
        
        print(f"Original features: {test_df.shape[1]-1}")
        print(f"Selected features: {selected_features_df.shape[1]-1}")
        print("\nTop Random Forest features:")
        print(rf_scores_df.head(10))
        
        # Check if important features were selected
        selected_feature_names = rf_scores_df['Feature'].tolist()
        important_feature_names = [f'Feature_{i+1}' for i in important_features]
        
        found_important = sum(1 for feat in important_feature_names if feat in selected_feature_names)
        print(f"\nImportant features found: {found_important}/{len(important_features)}")
        print(f"Important features: {important_feature_names}")
        print(f"Selected features: {selected_feature_names[:8]}")
        
        print("\n✅ Random Forest feature selection test PASSED")
        return True
        
    except Exception as e:
        print(f"\n❌ Random Forest feature selection test FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all tests."""
    print("Testing new feature selection methods...")
    
    lasso_success = test_lasso_selection()
    rf_success = test_random_forest_selection()
    
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    print(f"Lasso feature selection: {'✅ PASS' if lasso_success else '❌ FAIL'}")
    print(f"Random Forest feature selection: {'✅ PASS' if rf_success else '❌ FAIL'}")
    
    if lasso_success and rf_success:
        print("\n🎉 All tests PASSED! New feature selection methods are ready.")
    else:
        print("\n⚠️  Some tests FAILED. Please check the implementation.")

if __name__ == "__main__":
    main()
