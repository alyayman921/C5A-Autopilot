#include "fileHandling.h"

double B[65] = {0};

#ifdef USE_XLSX
void fileHandling(const char *filename) {
    xlsxioreader xlsxioread;
    xlsxioreadersheet sheet;
    char *cell_value;

    xlsxioread = xlsxioread_open(filename);
    if (xlsxioread == NULL) {
        fprintf(stderr, "Error: Could not open '%s'\n", filename);
        fprintf(stderr, "Make sure the file exists and is a valid .xlsx file\n");
        return;
    }
    sheet = xlsxioread_sheet_open(xlsxioread, NULL, XLSXIOREAD_SKIP_EMPTY_ROWS);
    if (sheet == NULL) {
        fprintf(stderr, "Error: Could not open worksheet\n");
        xlsxioread_close(xlsxioread);
        return;
    }
    int row_count = 0;
    while (xlsxioread_sheet_next_row(sheet)) {
        row_count++;
        int col_count = 0;
        while ((cell_value = xlsxioread_sheet_next_cell(sheet)) != NULL) {
            if (col_count == 1) {
                handling(row_count, cell_value, B);
            }
            col_count++;
            xlsxioread_free(cell_value);
        }
    }
    xlsxioread_sheet_close(sheet);
    xlsxioread_close(xlsxioread);
}
#endif

void readcsv(const char *filename) {
    FILE *ptrCSV = fopen(filename, "r");
    if (ptrCSV == NULL) {
        fprintf(stderr, "Error: Could not open '%s'\n", filename);
        return;
    }
    char line[1024];
    int row = 0;
    while (row < 63 && fgets(line, sizeof(line), ptrCSV) != NULL) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        char field[256] = {0};
        int idx = 0;
        const char *p = line;
        int found = 0;
        while (*p && idx <= 1) {
            int in_quote = 0;
            const char *start = p;
            while (*p) {
                if (*p == '"') {
                    in_quote = !in_quote;
                } else if (*p == ',' && !in_quote) {
                    break;
                }
                p++;
            }
            if (idx == 1) {
                const char *q = start;
                size_t o = 0;
                while (q < p && o < sizeof(field) - 1) {
                    if (*q == '"' || *q == ',') {
                        q++;
                        continue;
                    }
                    field[o++] = *q;
                    q++;
                }
                field[o] = '\0';
                found = 1;
                break;
            }
            idx++;
            if (*p == ',') {
                p++;
            } else {
                break;
            }
        }
        int store = row + 2;
        if (found) {
            B[store] = atof(field);
        } else {
            B[store] = 0.0;
        }
        row++;
    }
    fclose(ptrCSV);
}

void handling(int row, char current_cell[], double data[]) {
    data[row] = atof(current_cell);
}
