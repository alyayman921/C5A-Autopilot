#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <xlsxio_read.h>
#include <math.h>

inline char temp[21];

typedef struct {
  double B[65];
} raw_data;

inline void handling(int row,char current_cell[],double doubleed[]){
    int i=0;
    while (current_cell[i]!='\0'){
        temp[i]=current_cell[i];
        i++;
    }
    temp[i]='\0';
    doubleed[row]=atof(current_cell);
}


inline raw_data readxlsx(const char *filename) {
    raw_data data = {};

    xlsxioreader xlsxioread;
    xlsxioreadersheet sheet;
    char *cell_value;

    //printf("Opening file: %s\n", filename);
    xlsxioread = xlsxioread_open(filename);
    if (xlsxioread == NULL) {
        fprintf(stderr, "Error: Could not open '%s'\n", filename);
        fprintf(stderr, "Make sure the file exists and is a valid .xlsx file\n");
    }
    
    /* Open the first sheet */
    sheet = xlsxioread_sheet_open(xlsxioread, NULL, XLSXIOREAD_SKIP_EMPTY_ROWS);
    if (sheet == NULL) {
        fprintf(stderr, "Error: Could not open worksheet\n");
        xlsxioread_close(xlsxioread);
    }
    
    //printf("\nSheet contents:\n");
    //printf("----------------\n");
    
    int row_count = 0;
    while (xlsxioread_sheet_next_row(sheet)) {

        row_count++;
        int col_count = 0;
        
        while ((cell_value = xlsxioread_sheet_next_cell(sheet)) != NULL) {
            if (col_count==1){
              handling(row_count,cell_value,data.B);
            }
            col_count++;
            xlsxioread_free(cell_value); 
        }
        
        if (col_count == 0) {
            //printf("Row %d: (empty row)\n", row_count);
        }
    }
    //printf("Read Successful\n");

    /* Cleanup */
    xlsxioread_sheet_close(sheet);
    xlsxioread_close(xlsxioread);
    
    return data;
}