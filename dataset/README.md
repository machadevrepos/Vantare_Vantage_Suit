# Vantare Bicep Curl Dataset v1

This folder is the labeled upload source for Google Colab. The original files
in `converted_csv` remain unchanged.

## Labels

| Class ID | Folder | Sessions | Meaning |
| --- | --- | --- | --- |
| 0 | `correct` | 01, 02 | Proper bicep curl |
| 1 | `incomplete_range` | 03, 04 | Curl with incomplete range of motion |
| 2 | `elbow_movement` | 05, 06 | Curl with the elbow moving forward/backward |

## Sensor placement

- N2: wrist/distal forearm
- N3: elbow region
- N4: upper arm near the shoulder

Each session contains BNO85 CSV, ICM45686 CSV, and metadata JSON files for
N2, N3, and N4. Master files are intentionally excluded because the first
model uses only the three arm nodes.

## Validation rule

Files from one session must stay together. Do not randomly split CSV rows or
overlapping windows between training and testing. With the current six
sessions, use sessions 01/03/05 versus 02/04/06, then reverse the split.

## Colab upload

Compress the entire `dataset` folder and upload the archive to Colab. Preserve
the folder names because they are the authoritative class labels.

