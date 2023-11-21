#include "libcanard/canard.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/node/ExecuteCommand_1_1.h"
#include "uavcan/_register/List_1_0.h"
#include "uavcan/_register/Access_1_0.h"

#include "main.h"
#include "communication.h"

/* USER CODE BEGIN PV */
CanardInstance 	canard;		// This is the core structure that keeps all of the states and allocated resources of the library instance
CanardTxQueue 	queue;		// Prioritized transmission queue that keeps CAN frames destined for transmission via one CAN interface

// buffer for serialization of heartbeat message
size_t hbeat_ser_buf_size = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
uint8_t hbeat_ser_buf[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

// Wrappers for using memory allocator with libcanard
static void *memAllocate(CanardInstance *const canard, const size_t amount);
static void memFree(CanardInstance *const canard, void *const pointer);

// Application-specific function prototypes
void process_canard_TX_queue(void);
void processReceivedTransfer(int32_t redundant_interface_index, CanardRxTransfer *transfer);

CanardRxSubscription reg_access_subscription;
CanardRxSubscription reg_list_subscription;

void uavcan_setup(void)
{
  canard = canardInit(&memAllocate, &memFree);	// Initialization of a canard instance
  
  canard.node_id = 9;//config->node_id;

  // Limit the size of the queue at 100 frames , set MTU CANARD_MTU_CAN_FD = 64 bytes
  queue = canardTxInit(	100, CANARD_MTU_CAN_FD);  

  if( canardRxSubscribe(        &canard,
                                CanardTransferKindRequest,
                                uavcan_register_Access_1_0_FIXED_PORT_ID_,
                                uavcan_register_Access_Request_1_0_EXTENT_BYTES_,
                                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                &reg_access_subscription) != 1 ){ Error_Handler(); }  

  if( canardRxSubscribe(        &canard,
                                CanardTransferKindRequest,
                                uavcan_register_List_1_0_FIXED_PORT_ID_,
                                uavcan_register_List_Request_1_0_EXTENT_BYTES_,
                                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                &reg_list_subscription) != 1 ){ Error_Handler(); }  
}

void uavcan_spin(void)
{
  static uint8_t my_message_transfer_id = 0;
  static uint64_t timestamp = 0;

  if( micros() > timestamp + 1000000u )
  {
    timestamp = micros();
    // Create a heartbeat message
    uavcan_node_Heartbeat_1_0 test_heartbeat = {.uptime = micros()/1000000u,
                                                .health = {uavcan_node_Health_1_0_NOMINAL},
                                                .mode = {uavcan_node_Mode_1_0_OPERATIONAL}};

    // Serialize the heartbeat message
    if (uavcan_node_Heartbeat_1_0_serialize_(&test_heartbeat, hbeat_ser_buf, &hbeat_ser_buf_size) < 0)
    {
      Error_Handler();
    }

    // Create a transfer for the heartbeat message
    const CanardTransferMetadata transfer_metadata = {.priority = CanardPriorityNominal,
                                                      .transfer_kind = CanardTransferKindMessage,
                                                      .port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
                                                      .remote_node_id = CANARD_NODE_ID_UNSET,
                                                      .transfer_id = my_message_transfer_id,};

    if(canardTxPush(&queue,               	// Call this once per redundant CAN interface (queue)
                    &canard,
                    micros() + 500000u,     					// Zero if transmission deadline is not limited.
                    &transfer_metadata,
                    hbeat_ser_buf_size,		// Size of the message payload (see Nunavut transpiler)
                    hbeat_ser_buf) < 0 )
                    {
                      Error_Handler();
                    }
    
    // Increment the transfer_id variable
    my_message_transfer_id++;
  }
  
  FDCAN_ProcessFifo();
  process_canard_TX_queue();
}

void execute_command_callback(CanardRxTransfer *transfer);
void register_list_callback(CanardRxTransfer *transfer);
void register_access_callback(CanardRxTransfer *transfer);

void process_canard_TX_queue(void)
{
  // Look at top of the TX queue of individual CAN frames
  for (const CanardTxQueueItem* ti = NULL; (ti = canardTxPeek(&queue)) != NULL;)
  {
    if ((0U == ti->tx_deadline_usec) || (ti->tx_deadline_usec > micros()))  // Check the deadline.
    { 
      if( !FDCAN_PushFrame( ti->frame.extended_can_id, ti->frame.payload_size, (uint8_t *)ti->frame.payload ) )
      {
        // After the frame is transmitted or if it has timed out while waiting, pop it from the queue and deallocate:
        canard.memory_free(&canard, canardTxPop(&queue, ti));
      }
    }
    else
    {
      // the message expired
      canard.memory_free(&canard, canardTxPop(&queue, ti));
    }
  }
}

void FDCAN_RxProcessing( uint32_t id, uint8_t data_size, uint8_t *data)
{
  CanardFrame rxf;
  
  rxf.extended_can_id = id;
  rxf.payload_size = (size_t)data_size;
  rxf.payload = (void*)data;

  CanardRxTransfer transfer;

  int8_t result = canardRxAccept(       &canard,
                                        micros(),
                                        &rxf,
                                        0,
                                        &transfer,
                                        NULL);

  if (result < 0)
  {
    // An error has occurred: either an argument is invalid or we've ran out of memory.
    // It is possible to statically prove that an out-of-memory will never occur for a given application if
    // the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
    // Reception of an invalid frame is NOT an error.
  }
  else if (result == 1)
  {
    processReceivedTransfer(0, &transfer);              // A transfer has been received, process it.
    canard.memory_free(&canard, transfer.payload);      // Deallocate the dynamic memory afterwards.
  }
  else
  {
    // Nothing to do.
    // The received frame is either invalid or it's a non-last frame of a multi-frame transfer.
    // Reception of an invalid frame is NOT reported as an error because it is not an error.
  }
  
  return ;
}

void processReceivedTransfer(int32_t redundant_interface_index, CanardRxTransfer *transfer)
{
  if( transfer->metadata.port_id == uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_)
  {
    // Got command request!
    execute_command_callback(transfer);
  }
  else if( transfer->metadata.port_id == uavcan_register_List_1_0_FIXED_PORT_ID_)
  {
    // Got register list request! 
    register_list_callback(transfer);
  }
  else if( transfer->metadata.port_id == uavcan_register_Access_1_0_FIXED_PORT_ID_)
  {
    // Got register list request! 
    register_access_callback(transfer);
  }
  else
  {
    // Received unknown completed transfer
  }
  
  return ;
}

void execute_command_callback(CanardRxTransfer *transfer)
{
  uavcan_node_ExecuteCommand_Request_1_1 request;
  uavcan_node_ExecuteCommand_Response_1_1 response = {0};
  
  size_t request_ser_buf_size = uavcan_node_ExecuteCommand_Request_1_1_EXTENT_BYTES_;

  if( uavcan_node_ExecuteCommand_Request_1_1_deserialize_(&request, transfer->payload, &request_ser_buf_size ) < 0)
  {
    Error_Handler();
  }
  
  if( request.command == uavcan_node_ExecuteCommand_Request_1_1_COMMAND_RESTART )
  {
    //MCU_restart = 1;
    response.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
  }
  else
  {
    response.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_BAD_COMMAND;
  }

  uint8_t c_serialized[uavcan_node_ExecuteCommand_Response_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
  size_t c_serialized_size = sizeof(c_serialized);

  if ( uavcan_node_ExecuteCommand_Response_1_1_serialize_(&response, &c_serialized[0], &c_serialized_size) < 0)
  {
    Error_Handler();
  }

  const CanardTransferMetadata transfer_metadata = {    .priority       = CanardPriorityNominal,
                                                        .transfer_kind  = CanardTransferKindResponse,
                                                        .port_id        = transfer->metadata.port_id,
                                                        .remote_node_id = transfer->metadata.remote_node_id,
                                                        .transfer_id    = transfer->metadata.transfer_id };

  int32_t debug =  canardTxPush(        &queue,               	// Call this once per redundant CAN interface (queue)
                                        &canard,
                                        micros() + 500000u,     					// Zero if transmission deadline is not limited.
                                        &transfer_metadata,
                                        c_serialized_size,		// Size of the message payload (see Nunavut transpiler)
                                        c_serialized);
}

void register_list_callback(CanardRxTransfer *transfer)
{
  uavcan_register_List_Request_1_0 request;
  size_t request_ser_buf_size = uavcan_register_List_Request_1_0_EXTENT_BYTES_;

  if( uavcan_register_List_Request_1_0_deserialize_(&request, transfer->payload, &request_ser_buf_size ) < 0)
  {
    Error_Handler();
  }

  uavcan_register_List_Response_1_0 response = {0};

  char led1_register_name[] = "user.led1";
  char led2_register_name[] = "user.led2";

  if( request.index == 0 )
  {
    memcpy(&response.name.name.elements, led1_register_name, sizeof(led1_register_name));
    response.name.name.count = sizeof(led1_register_name);
  }
  else if( request.index == 1 )
  {
    memcpy(&response.name.name.elements, led2_register_name, sizeof(led2_register_name));
    response.name.name.count = sizeof(led2_register_name);
  }
  else
  {
    response.name.name.elements[0] = '\0';
    response.name.name.count = 0;
  }

  uint8_t c_serialized[uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
  size_t c_serialized_size = sizeof(c_serialized);

  if ( uavcan_register_List_Response_1_0_serialize_(&response, &c_serialized[0], &c_serialized_size) < 0)
  {
    Error_Handler();
  }

  const CanardTransferMetadata transfer_metadata = {    .priority       = CanardPriorityNominal,
                                                        .transfer_kind  = CanardTransferKindResponse,
                                                        .port_id        = transfer->metadata.port_id,
                                                        .remote_node_id = transfer->metadata.remote_node_id,
                                                        .transfer_id    = transfer->metadata.transfer_id };
  
  int32_t debug =  canardTxPush(        &queue,               	// Call this once per redundant CAN interface (queue)
                                        &canard,
                                        micros() + 500000u,     					// Zero if transmission deadline is not limited.
                                        &transfer_metadata,
                                        c_serialized_size,		// Size of the message payload (see Nunavut transpiler)
                                        c_serialized);
}

void LED1_access_handler(uavcan_register_Value_1_0 * req_value, uavcan_register_Value_1_0 * res_value );
void LED2_access_handler(uavcan_register_Value_1_0 * req_value, uavcan_register_Value_1_0 * res_value );
void LED3_access_handler(uavcan_register_Value_1_0 * req_value, uavcan_register_Value_1_0 * res_value );
void offset_access_handler(uavcan_register_Value_1_0 * req_value, uavcan_register_Value_1_0 * res_value );

void register_access_callback(CanardRxTransfer *transfer)
{
  uavcan_register_Access_Request_1_0 request;
  uavcan_register_Access_Response_1_0 response = {0};
  
  size_t request_ser_buf_size = uavcan_register_Access_Request_1_0_EXTENT_BYTES_;

  if( uavcan_register_Access_Request_1_0_deserialize_(&request, transfer->payload, &request_ser_buf_size ) < 0)
  {
    Error_Handler();
  }

  if( !strncmp( (char const *)"user.PC7", (char const *)request.name.name.elements, request.name.name.count) )
  {
    LED1_access_handler(&request.value, &response.value);
  }
  else if( !strncmp( (char const *)"user.PC8", (char const *)request.name.name.elements, request.name.name.count) )
  {
    LED2_access_handler(&request.value, &response.value);
  }
  else if( !strncmp( (char const *)"user.PC9", (char const *)request.name.name.elements, request.name.name.count) )
  {
    LED3_access_handler(&request.value, &response.value);
  }  
  else
  {
    // access to non-existent register
    response.value._tag_= 0;
  }

  response.timestamp.microsecond = micros(); // taken along with register READING
  response._mutable = 1;
  response.persistent = 0;
  
  uint8_t c_serialized[uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
  size_t c_serialized_size = sizeof(c_serialized);

  if ( uavcan_register_Access_Response_1_0_serialize_(&response, &c_serialized[0], &c_serialized_size) < 0)
  {
    Error_Handler();
  }

  const CanardTransferMetadata transfer_metadata = {    .priority       = CanardPriorityNominal,
                                                        .transfer_kind  = CanardTransferKindResponse,
                                                        .port_id        = transfer->metadata.port_id,
                                                        .remote_node_id = transfer->metadata.remote_node_id,
                                                        .transfer_id    = transfer->metadata.transfer_id };
  
  int32_t debug =  canardTxPush(        &queue,               	// Call this once per redundant CAN interface (queue)
                                        &canard,
                                        micros() + 500000u,     					// Zero if transmission deadline is not limited.
                                        &transfer_metadata,
                                        c_serialized_size,		// Size of the message payload (see Nunavut transpiler)
                                        c_serialized);  
}

void LED1_access_handler(uavcan_register_Value_1_0 * req_value, uavcan_register_Value_1_0 * res_value )
{
  if( req_value->_tag_ != 0 ) // is it write access ?
  {
    if( req_value->integer8.value.elements[0] == 1 )
    {
      LL_GPIO_SetOutputPin(PC7_GPIO_Port, PC7_Pin); // reset channels output control
    }
    else if( req_value->integer8.value.elements[0] == 0 )
    {
      LL_GPIO_ResetOutputPin(PC7_GPIO_Port, PC7_Pin); // reset channels output control
    }
  }

  // return register value
  res_value->_tag_= 7;
  res_value->integer8.value.count = 1;
  res_value->integer8.value.elements[0] = LL_GPIO_IsOutputPinSet(PC7_GPIO_Port, PC7_Pin);
}

void LED2_access_handler(uavcan_register_Value_1_0 * req_value, uavcan_register_Value_1_0 * res_value )
{
  if( req_value->_tag_ != 0 ) // is it write access ?
  {
    if( req_value->integer8.value.elements[0] == 1 )
    {
      LL_GPIO_SetOutputPin(PC8_GPIO_Port, PC8_Pin); // reset channels output control
    }
    else if( req_value->integer8.value.elements[0] == 0 )
    {
      LL_GPIO_ResetOutputPin(PC8_GPIO_Port, PC8_Pin); // reset channels output control
    }
  }

  // return register value
  res_value->_tag_= 7;
  res_value->integer8.value.count = 1;
  res_value->integer8.value.elements[0] = LL_GPIO_IsOutputPinSet(PC8_GPIO_Port, PC8_Pin);
}

void LED3_access_handler(uavcan_register_Value_1_0 * req_value, uavcan_register_Value_1_0 * res_value )
{
  if( req_value->_tag_ != 0 ) // is it write access ?
  {
    if( req_value->integer8.value.elements[0] == 1 )
    {
      LL_GPIO_SetOutputPin(PC9_GPIO_Port, PC9_Pin); // reset channels output control
    }
    else if( req_value->integer8.value.elements[0] == 0 )
    {
      LL_GPIO_ResetOutputPin(PC9_GPIO_Port, PC9_Pin); // reset channels output control
    }
  }

  // return register value
  res_value->_tag_= 7;
  res_value->integer8.value.count = 1;
  res_value->integer8.value.elements[0] = LL_GPIO_IsOutputPinSet(PC9_GPIO_Port, PC9_Pin);
}

// allocate dynamic memory of desired size in bytes
static void *memAllocate(CanardInstance *const canard, const size_t amount)
{
  (void)canard;

  void * pointer = NULL;
  
  uint32_t primask_bit = __get_PRIMASK();       // backup PRIMASK bit
  __disable_irq();                              // Disable all interrupts by setting PRIMASK bit on Cortex
  
    pointer = malloc(amount);
    
  __set_PRIMASK(primask_bit);                   // Restore PRIMASK bit    
  
  return pointer;
}

// free allocated memory
static void memFree(CanardInstance *const canard, void *const pointer)
{
  (void)canard;
  
  uint32_t primask_bit = __get_PRIMASK();       // backup PRIMASK bit
  __disable_irq();                              // Disable all interrupts by setting PRIMASK bit on Cortex
  
    free(pointer);
    
  __set_PRIMASK(primask_bit);                   // Restore PRIMASK bit  
}