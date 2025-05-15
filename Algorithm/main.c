#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NUMPIC 8
#define ROWS 12
#define COLS 16
#define SIZE_THRESHOLD 3

int frame[NUMPIC][ROWS][COLS];
int threshold_frame[ROWS][COLS];
int visited[ROWS][COLS];

// Directions: up, down, left, right
int dr[] = {-1, 1, 0, 0};
int dc[] = {0, 0, -1, 1};

void generate_temperature_data() {
    for (int i = 0; i < NUMPIC; i++) {
        for (int r = 0; r < ROWS; r++) {
            for (int c = 0; c < COLS; c++) {
                frame[i][r][c] = 20 + rand() % 15;
            }
        }
    }
}

void compute_median_frame() {
    int values[NUMPIC];
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            for (int i = 0; i < NUMPIC; i++) {
                values[i] = frame[i][r][c];
            }
            // Bubble sort
            for (int i = 0; i < NUMPIC - 1; i++) {
                for (int j = 0; j < NUMPIC - i - 1; j++) {
                    if (values[j] > values[j + 1]) {
                        int temp = values[j];
                        values[j] = values[j + 1];
                        values[j + 1] = temp;
                    }
                }
            }
            threshold_frame[r][c] = values[NUMPIC / 2]; // Median
        }
    }
}

void apply_threshold() {
    for (int i = 0; i < NUMPIC; i++) {
        for (int r = 0; r < ROWS; r++) {
            for (int c = 0; c < COLS; c++) {
                if (frame[i][r][c] < threshold_frame[r][c]) {
                    frame[i][r][c] = 0;
                }
            }
        }
    }
}

int dfs(int i, int r, int c) {
    if (r < 0 || r >= ROWS || c < 0 || c >= COLS || visited[r][c] || frame[i][r][c] == 0)
        return 0;
    visited[r][c] = 1;
    int size = 1;
    for (int d = 0; d < 4; d++) {
        int nr = r + dr[d];
        int nc = c + dc[d];
        if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS) {
            size += dfs(i, nr, nc);
        }
    }
    return size;
}

int count_people(int i) {
    memset(visited, 0, sizeof(visited));
    int count = 0;
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            if (!visited[r][c] && frame[i][r][c] > 0) {
                int size = dfs(i, r, c);
                if (size >= SIZE_THRESHOLD) {
                    count++;
                }
            }
        }
    }
    return count;
}

int mode(int arr[], int n) {
    int max_val = 0;

    // Find max to size frequency array safely
    for (int i = 0; i < n; i++) {
        if (arr[i] > max_val) max_val = arr[i];
    }

    int *freq = (int *)calloc(max_val + 1, sizeof(int));
    if (!freq) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(1);
    }

    for (int i = 0; i < n; i++) {
        freq[arr[i]]++;
    }

    int maxFreq = 0, modeVal = 0;
    for (int i = 0; i <= max_val; i++) {
        if (freq[i] > maxFreq) {
            maxFreq = freq[i];
            modeVal = i;
        }
    }

    free(freq);
    return modeVal;
}

int main() {
    generate_temperature_data();
    compute_median_frame();
    apply_threshold();

    int numPeople[NUMPIC];
    for (int i = 0; i < NUMPIC; i++) {
        for(int j = 0; j < ROWS; j++)
        {
            for(int k = 0; k < COLS; k++)
            {
                printf("%d ", frame[NUMPIC][j][k]);
            }

            printf("\n");
        }
        numPeople[i] = count_people(i);
        printf("Frame %d: detected %d people\n", i, numPeople[i]);
    }

    int pplCount = mode(numPeople, NUMPIC);
    printf("\nEstimated number of people in the room: %d\n", pplCount);

    return 0;
}