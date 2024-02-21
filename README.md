Для HP PWR раскомментируй строчку:

#define curr_sns_inver

Если на плате есть перемычка от ножки PA5 на модуле, раскомментируй блок:

#undef BUS_CTl_Pin

#undef BUS_CTl_GPIO_Port

#define BUS_CTl_Pin LL_GPIO_PIN_5

#define BUS_CTl_GPIO_Port GPIOA
