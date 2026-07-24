#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#define n_floats 4
#define size_float 5 // in received bytes
#define string_size n_floats*size_float
TIM_HandleTypeDef htim1;
// UART_HandleTypeDef huart1;

void SystemClock_Config(void);
void empty_string();
int read_string();
void write_string();
void write_floats();
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
extern int rx_receive(uint8_t *out);
// void handle_floats();
void handle_ints();
void write_ack();


// volatile uint32_t seconds_counter = 0;
int timeout=10; // 10 ms 
uint8_t start_char='!';
uint8_t terminating_char='$';
uint8_t byte_received=0;
char string_received[string_size];
char sent_string[string_size+2];
uint8_t* rx_rec=&byte_received,x;
char one_float[size_float]={0};
int inputs[n_floats]={0.0};

int main(void){
{  HAL_Init();
SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  // Pull USB Pin 12 so it resets normally
  GPIO_InitStruct.Pin   = GPIO_PIN_12;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(200);
  MX_USB_DEVICE_Init();
  HAL_Delay(1000);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_Delay(200);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  x=sizeof(float);

  //CDC_Transmit_FS((uint8_t *)"Code Started\n", 13);
  HAL_TIM_Base_Start_IT(&htim1);
}
  while (1)
  {
    // if(rx_receive(rx_rec) && *rx_rec==start_char){ // if it detects start character
    //   // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    //   while(!read_string()){} // return 1 when there's value received
    //   // string_received[11]=(int8_t)string_received[11]+1;
    //   // write_string();
    //   inputs[0]-=1.1;
    //   inputs[1]-=1.1;
    //   write_floats();
    //   HAL_Delay(10); 
    // }
    // x=sizeof(float);
    // printf("%i",x);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // HAL_Delay(500);
    if(rx_receive(rx_rec) && *rx_rec==start_char){
      while(!read_string()){}
        handle_ints();
        if(inputs[0]==12345){
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
          write_ack();
        }else if(inputs[0]==54321){
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
          HAL_Delay(2000);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
        }
        // write_string();
      // HAL_Delay(10);
    }
  }
}

void empty_string(){
        for (int i=0;i<string_size;i++){
            string_received[i]='0';
        }
        for (int i=0;i<string_size+2;i++){
            sent_string[i]='0';
        }
}

int read_string(){
        int i=0;
        empty_string();
        do{
            if(rx_receive(rx_rec)){
                string_received[i]=*rx_rec;
                i++;
            }
            if (*rx_rec==terminating_char){
              HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
              return 1;
            }
        }while(i<20);
          return 0; // limit reached
}

void write_string(){
  sent_string[0]=start_char;
  for(int i=1;i<string_size+1;i++){
    sent_string[i]=string_received[i-1];
}
  sent_string[string_size+1]=terminating_char;
  CDC_Transmit_FS((uint8_t*)sent_string, string_size);
}
void write_ack(){
  char sent_string[2];
  sent_string[0]=start_char;sent_string[1]=terminating_char;
  CDC_Transmit_FS((uint8_t*)sent_string, 2);
}

// void handle_floats(){
//   int i=0,j=0,j0=0;
//   for(i;i<n_floats;i++){
//     j0=j;
//     for(j;j<size_float;j++){
//       one_float[j-j0]=string_received[j];
//     }
//     inputs[i]=atof(one_float);
//   }
// }
void handle_ints(){
  int i=0,j=0,j0=0;
  int one_int[size_float];
  for(i;i<n_floats;i++){
    j0=j;
    for(j;j<size_float;j++){
      one_int[j-j0]=string_received[j];
    }
    inputs[i]=(int)one_int;
  }
}

void SystemClock_Config(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  uint32_t retries = 3;
  while (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK && retries--) {
    HAL_Delay(10);
  }
  if (retries == 0) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void){

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  /* TIM1 is on APB2, whose clock is not divided (APB2CLKDivider = DIV1),
     so TIM1's clock equals SYSCLK = 72 MHz.
     Prescaler 7199 + Period 9999 -> 72MHz / 7200 / 10000 = 1 Hz interrupt */
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Disabled external slave-mode triggering so TIM1 free-runs on its
     own internal clock instead of waiting on ITR0 */
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

void Error_Handler(void){
  __disable_irq();
  // HAL_Delay(500);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif 