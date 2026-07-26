#include "mySerial.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
//extern int timeout;
extern int rx_receive(uint8_t *out);
extern uint8_t start_char;
extern uint8_t terminating_char;

extern uint8_t* rx_rec;
char str_received[string_size];
char sent_string[string_size+2];

extern int input_i[n_ints];
extern float input_f[n_floats];

void empty_string(void){
        for (int i=0;i<string_size;i++){
            str_received[i]='0';
        }
        for (int i=0;i<string_size+2;i++){
            sent_string[i]='0';
        }
}

int read_string(void){
        int i=0;
        empty_string();
        do{
            if(rx_receive(rx_rec)){
                str_received[i]=*rx_rec;
                i++;
            }
            if (*rx_rec==terminating_char){
              return 1;
            }
        }while(i<string_size+1);
          return 0; // string limit limit reached
}

void write_string(void){
  sent_string[0]=start_char;
  for(int i=1;i<string_size+1;i++){
    sent_string[i]=str_received[i-1];
  }
  sent_string[string_size+1]=terminating_char;
  CDC_Transmit_FS((uint8_t*)sent_string, string_size);
}

int write_ack(void){
  char acknowledge[2];
  acknowledge[0]=start_char;acknowledge[1]=terminating_char;
  return (CDC_Transmit_FS((uint8_t*)acknowledge, 2));
}

void handle_ints(){
    char buffer[size_int + 1];
    for(int i = 0; i < n_ints; i++){
        for(int j = 0; j < size_int; j++){
            buffer[j] = str_received[i*size_int + j];
        }
        buffer[size_int] = '\0';
        input_i[i] = atoi(buffer);
        printf("%d\n", input_i[i]);
    }
}