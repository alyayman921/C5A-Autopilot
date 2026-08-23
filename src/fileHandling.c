#include "fileHandling.h"
#include <stdio.h>
#include <stdlib.h>

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
                B[row_count] = atof(cell_value);
            }
            col_count++;
            xlsxioread_free(cell_value);
        }
    }
    xlsxioread_sheet_close(sheet);
    xlsxioread_close(xlsxioread);
}
#endif
