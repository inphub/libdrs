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

// ����������� ������ ����� ��������
#define DRS_PROTO_DATA_MEM_FREE_AFTER 0x00000001

/**
  * @struct drs_proto_data
  * @brief ��������� �� ������ ��� ����� ���� ��������
  *
  */
typedef struct drs_proto_data{
    drs_proto_data_type_t type;
    union{
        struct{
            union{
                void * ptr;     /** @brief ��������� �� ���� ������ */
                byte_t * ptr_byte;
            };
            size_t size;  /** @brief ������ ������ */
            size_t shift; /** @brief �������� ������� ������ ��� ������ ������������ ������  */
            int flags;
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
  * @brief �������� ������ ��������� �����
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

    pthread_rwlock_t out_rwlock;
    dap_list_t * out_first;  /** @brief ������ �� ������� �������� **/
    dap_list_t * out_last;   /** @brief ��������� �� ������� �������� **/

    pthread_rwlock_t in_rwlock;
    dap_list_t * in_first;   /** @brief ������ �� ������� ������ **/
    dap_list_t * in_last;    /** @brief ��������� �� ������� �������� **/

} drs_proto_t;
#define DRS_PROTO(a) ((drs_proto_t *) (a)->_inheritor)

extern const char g_inipath[];


int drs_proto_init(dap_server_t * a_server);
void drs_proto_deinit();
void drs_proto_out_add_mem(drs_proto_t * a_proto, void * a_data, size_t a_data_size, int a_flags);

void drs_proto_out_add_file(drs_proto_t * a_proto, int a_fd, bool a_close_fd_after);

