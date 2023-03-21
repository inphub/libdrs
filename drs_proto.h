/*
 * drs_proto.h
 *
 *  Created on: 25 October 2022
 *      Author: Dmitriy Gerasimov
 */
#pragma once
#include <dap_server.h>
#include <dap_events_socket.h>
#include <dap_list.h>

typedef enum drs_proto_data_type{ DATA_TYPE_MEMORY, DATA_TYPE_FILE } drs_proto_data_type_t;
/**
  * @struct drs_proto_data
  * @brief ��������� �� ������ ��� ����� ���� ��������
  *
  */
typedef struct drs_proto_data{
    drs_proto_data_type_t type;
    union{
        struct{
            const void * ptr;     /** @brief ��������� �� ���� ������ */
            size_t size;  /** @brief ������ ������ */
            size_t shift; /** @brief �������� ������� ������ ��� ������ ������������ ������  */
        } data;
        struct{
            int fd;
            off_t offset;
            bool close_fd_after;
        } file;
    };
} drs_proto_data_t;

/**
  * @enum drs_proto_state_in
  * @brief �������� ������ ��������� ������
  */
typedef enum drs_proto_state_in{
    STATE_IN_DEFAULT,
} drs_proto_state_in_t;

/**
  * @enum drs_proto_state_out
  * @brief �������� ������ ��������� ��������
  */
typedef enum drs_proto_state_out{
    STATE_OUT_DEFAULT,
} drs_proto_state_out_t;


/**
  * @struct drs_proto
  * @brief ������, ����������� ��� ��������� ����������
  */
typedef struct drs_proto{
    dap_events_socket_t * esocket; /** @brief �������� **/

    /**
     * @brief ��������� ���������/��������
     */
    struct{
        drs_proto_state_out_t out;
        drs_proto_state_out_t in;
    } state;

    dap_list_t * out_first;  /** @brief ������ �� ������� �������� **/
    dap_list_t * out_last;   /** @brief ��������� �� ������� �������� **/

    dap_list_t * in_first;   /** @brief ������ �� ������� ������ **/
    dap_list_t * in_last;    /** @brief ��������� �� ������� �������� **/

} drs_proto_t;
#define DRS_PROTO(a) ((drs_proto_t *) (a)->_inheritor)

extern const char g_inipath[];


int drs_proto_init(dap_server_t * a_server);
void drs_proto_deinit();
void drs_proto_out_add_mem(drs_proto_t * a_proto, const void * a_data, size_t a_data_size);
void drs_proto_out_add_file(drs_proto_t * a_proto, int a_fd, bool a_close_fd_after);

