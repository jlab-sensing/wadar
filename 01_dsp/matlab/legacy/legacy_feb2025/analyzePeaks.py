#!/usr/bin/env python
# coding: utf-8

# USAGE
# example: python analyzePeaks.py <path/fileName.xlsx>

import pandas as pd
import re
import matplotlib.pyplot as plt
import numpy as np
import sys
from datetime import datetime
import sys
import argparse

if len(sys.argv) < 2:
    xlsxFilename = 'corr.xlsx'
    path = '/home/bradley/Documents/Research/peak_detect/' + xlsxFilename
else:
    path = sys.argv[1]

# # Function declaration
def groupAnalysis(df, groupName):

    df_info = pd.DataFrame(index = df[groupName].unique(), columns = ['count', 'meanPeakError', 'meanAbsPeakError', 'peakErrorWithinT', 'stdPeakError', '5%', '95%'])

    df_grouped = df[[groupName] + ['peakErrors_1', 'peakErrorsAbs_1', 'peakErrorsWithinT_1']].groupby(groupName)

    df_info['meanPeakError'] = df_grouped['peakErrors_1'].mean()
    df_info['stdPeakError'] = df_grouped['peakErrors_1'].std()
    df_info['meanAbsPeakError'] = df_grouped['peakErrorsAbs_1'].mean()
    df_info['peakErrorWithinT'] = df_grouped['peakErrorsWithinT_1'].mean()
    df_info['count'] = df_grouped['peakErrors_1'].count()
    df_info['5%'] = df_grouped['peakErrors_1'].quantile(0.05)
    df_info['95%'] = df_grouped['peakErrors_1'].quantile(0.95)

    return df_info.sort_index()


# # get the dataframe and do some processing
df = pd.read_excel(path)

# drop null values
df = df.dropna()

# remove trailing number on captures
df['captureNames'] = df['captureNames'].apply(lambda s: re.sub(r'\d+$', '', s))

# define moisture levels in increments
df['moistureLevel'] = ((df['vwcTrue'] * 2 // 0.1) + 1).astype('int32')
df['moistureLevel'] = df['moistureLevel'].map({1: "0-0.1", 2: "0-0.1", 3: "0.1-0.2", 4: "0.1-0.2", 5: "0.2-0.25", \
        6: "0.25-0.3", 7: "0.3-0.35", 8: ">0.35", 9: ">0.35", 10: ">0.35"})

# define confidence levels in increments
df['confidenceLevel'] = (df['confidencePreds_1'] // 0.1).astype('int32')
df['confidenceLevel'] = df['confidenceLevel'].map({0: "0-0.1",\
                                              1: "0.1-0.2",\
                                              2: "0.2-0.3",\
                                              3: "0.3-0.4",\
                                              4: "0.4-0.5",\
                                              5: "0.5-0.6",\
                                              6: "0.6-0.7",\
                                              7: "0.7-0.8",\
                                              8: "0.8-0.9",\
                                              9: "0.9-1"})

# calculate absolute value of peak error
df['peakErrorsAbs_1'] = abs(df['peakErrors_1'])

# define whether the experiment was active or passive
df['tagType'] = df['expNames'].apply(lambda x: 'passive' if ('passive' in x) else 'active')

# determine if auto peak within t bins of manual peak
t = 15
df['peakErrorsWithinT_1'] = np.where(df['peakErrors_1'].abs() <= t, True, False)

# # How close are we to the correct bin, on average?

# print('mean, std for error on bin prediction for template 1 = {}, {}\n'      .format(df['peakErrors_1'].mean(),df['peakErrors_1'].std()))

# hist = df.hist(column = 'peakErrors_1', bins = 100, figsize = (18,6))

# 5% and 95%
# print('Roughly 5% of bin predictions are more than than {} below the measured bin'      .format(-1*df['peakErrors_1'].quantile(.05).round(0)))
# print('Roughly 5% of bin predictions are more than than {} above the measured bin'      .format(df['peakErrors_1'].quantile(.95).round(0)))

# # Group Analysis

# Analysis by Moisture Level
#outputFilename = path.rstrip(".xlsx") + ".txt"

#with open(outputFilename, 'a') as fo:
#    now = datetime.now()
#    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
#    fo.write(dt_string + "\n\n")
#    fo.write(groupAnalysis(df, 'moistureLevel').__repr__() + "\n\n")
#    fo.write(groupAnalysis(df, 'expNames').__repr__() + "\n\n")
#    fo.write(groupAnalysis(df, 'tagType').__repr__() + "\n\n")
print('moisture level...\n')
print(groupAnalysis(df, 'moistureLevel'))
print('experiment category...\n')
print(groupAnalysis(df, 'expNames'))
print('tag type...\n')
print(groupAnalysis(df, 'tagType'))
print('confidence level...\n')
print(groupAnalysis(df, 'confidenceLevel'))
