#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#ifdef USE_XLSX
#include <xlsxio_read.h>
void fileHandling(const char *filename);
#endif

void readcsv(const char *filename);
void handling(int row, char current_cell[], double data[]);
