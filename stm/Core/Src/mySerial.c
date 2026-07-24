#include "mySerial.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
//extern int timeout;
extern int rx_receive(uint8_t *out);
extern uint8_t start_char;
extern uint8_t terminating_char;
char str_received[string_size];
char sent_string[string_size+2];
uint8_t* ptr_rx_byte;
int input[n_floats];

void set_serial_vars(uint8_t* byte_received,int inputs[n_floats]){
  ptr_rx_byte=byte_received;
  for(int n=0;n<n_floats;n++){
    input[n]=inputs[n];
  }
}

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
            if(rx_receive(ptr_rx_byte)){
                str_received[i]=*ptr_rx_byte;
                i++;
            }
            if (*ptr_rx_byte==terminating_char){
              HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
              return 1;
            }
        }while(i<20);
          return 0; // limit reached
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
  int i=0,j=0,j0=0;
  int one_int[size_float];
  for(i=0;i<n_floats;i++){
    j0=j;
    for(j;j<size_float;j++){
      one_int[j-j0]=str_received[j];
    }
    input[i]=*one_int;
  }
}
// void handle_floats(){
//   int i=0,j=0,j0=0;
//   for(i;i<n_floats;i++){
//     j0=j;
//     for(j;j<size_float;j++){
//       one_float[j-j0]=string_received[j];
//     }
//     input[i]=atof(one_float);
//   }
// }
