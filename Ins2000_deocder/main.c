#include <stdio.h>
#include"ins2000.h"

#define READ_CACHE_SIZE 4*1024

int main(int argc, char* argv[]) {
	if (argc > 1) {
		char* filename = argv[1];
		printf("Decode : %s \n", argv[1]);
		FILE* file = fopen(filename, "rb");
		if (file) {
			fseek(file, 0L, SEEK_END);
			int size = ftell(file);
			fseek(file, 0L, SEEK_SET);
			int  readcount = 0, i = 0, num = 0, step = 0,size_step = size / 100;
			double percent = 0.0;
			set_ins200_file_basename(filename);
			char read_cache[READ_CACHE_SIZE] = { 0 };
			while (!feof(file)) {
				readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
				//if (feof(file)) break;
				for (i = 0; i < readcount; i++) {
					input_ins2000_raw(read_cache[i]);
					num++;
					if (num > step*size_step || num == size) {
						percent = (double)num / (double)size * 100;
						printf("Process : %4.1f %%\r", percent);
						step++;
					}
				}
			}
			close_ins2000_all_file();
			fclose(file);
		}
	}
	return  0;
}