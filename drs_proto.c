/*
 * drs_proto.c
 *
 *  Created on: 25 October 2022
 *      Author: Dmitriy Gerasimov
 */

#include <sys/sendfile.h>
#include <fcntl.h>
#include <dap_common.h>

#include "drs_proto.h"
#include "drs_proto_cmd.h"

#define LOG_TAG "drs_proto"

const char g_inipath[]= "/media/card/config.ini";

static dap_server_t * s_server = NULL;

// Вызывается когда удаленная сторона прислала данные, доступные для чтения
static void s_callback_new (dap_events_socket_t * a_es,void * a_arg );
static void s_callback_delete (dap_events_socket_t * a_es,void * a_arg );
static void s_callback_read (dap_events_socket_t * a_es,void * a_arg );
static void s_callback_write (dap_events_socket_t * a_es,void * a_arg );
static void s_callback_write_finished(dap_events_socket_t * a_es, void * a_arg, int a_errno);

static inline void s_proto_out_add( drs_proto_t * a_proto, drs_proto_data_t * a_out);
static inline void s_proto_out_remove(drs_proto_t * a_proto, drs_proto_data_t * a_out);
/**
 * @brief Инициализация протокола
 * @param a_server Указатель на есокет сервер, чти входящие соединения мы будем обрабатывать
 * @return
 */
int drs_proto_init(dap_server_t * a_server)
{
    s_server = a_server;
    if ( s_server ){
        dap_events_socket_callbacks_t l_callbacks = {
            .new_callback = s_callback_new,
            .delete_callback = s_callback_delete,
            .read_callback = s_callback_read,
            .write_callback = s_callback_write,
            .write_finished_callback = s_callback_write_finished
        };
        s_server->client_callbacks = l_callbacks;
    }

    return 0;
}

/**
 * @brief drs_proto_deinit
 */
void drs_proto_deinit()
{

}

/**
 * @brief drs_proto_proc
 * @return
 */
int drs_proto_proc()
{
    return 0;
}

/**
 * @brief Добавляем участок памяти в очередь отправки
 * @param a_proto
 * @param a_data
 * @param a_data_size
 */
void drs_proto_out_add_mem(drs_proto_t * a_proto, const void * a_data, size_t a_data_size)
{
    // Добавляем данные на отправку в очередь
    drs_proto_data_t * l_out = DAP_NEW_Z(drs_proto_data_t);
    l_out->type = DATA_TYPE_MEMORY;
    l_out->data.ptr = a_data;
    l_out->data.size = a_data_size;
    s_proto_out_add(a_proto, l_out);
}

/**
 * @brief Добавляем файловый дескриптор в очередь отправки
 * @param a_proto
 * @param a_fd
 * @param a_close_fd_after
 */
void drs_proto_out_add_file(drs_proto_t * a_proto, int a_fd, bool a_close_fd_after)
{
    drs_proto_data_t * l_out = DAP_NEW_Z(drs_proto_data_t);
    l_out->type = DATA_TYPE_FILE;
    l_out->file.fd = a_fd;
    l_out->file.close_fd_after = a_close_fd_after;
    s_proto_out_add(a_proto, l_out);
}

/**
 * @brief s_proto_add_out
 * @param a_proto
 * @param a_out
 */
static inline void s_proto_out_add( drs_proto_t * a_proto, drs_proto_data_t * a_out)
{
    a_proto->out_first = dap_list_prepend( a_proto->out_first, a_out );
    if(a_proto->out_last == NULL)
        a_proto->out_last = a_proto->out_first;

    dap_events_socket_set_writable_unsafe(a_proto->esocket, true); // Выставляем флаг готовности отправки данных
}

/**
 * @brief s_proto_out_remove
 * @param a_proto
 */
static inline void s_proto_out_remove(drs_proto_t * a_proto, drs_proto_data_t * a_out)
{
    dap_list_t * l_out_last_prev =  a_proto->out_last->prev;
    // Очищаем память элемента очереди
    dap_list_free1(a_proto->out_last);
    DAP_DELETE(a_out);

    // Обновляем указатели на последний и, если нужно, первый элемент очереди
    a_proto->out_last = l_out_last_prev ;
    if ( a_proto->out_last == NULL)
        a_proto->out_first = NULL;
}


/**
 * @brief Вызывается при создании входящего соединения
 * @param a_es Есокет удаленного клиента
 * @param a_arg не используется
 */
static void s_callback_new (dap_events_socket_t * a_es,void * a_arg )
{
    UNUSED(a_arg);
    // Создаём экземпляр объекта-наследника
    drs_proto_t *l_proto = DAP_NEW_Z(drs_proto_t);
    l_proto->esocket = a_es;
    a_es->_inheritor = l_proto; // агрегируем его в родителя.
}

/**
 * @brief s_callback_read
 * @param a_es Есокет удаленного клиента
 * @param a_arg
 */
static void s_callback_read (dap_events_socket_t * a_es,void * a_arg )
{
    uint32_t l_cmd, * l_cmd_args;
    UNUSED(a_arg);
    if (a_es->buf_in_size < sizeof(uint32_t)) // Проверяем, достаточно ли у нас данных накопилось хотя бы для номера команды
        return; // вернёмся сюда тогда, когда клиент вышлет ещё данных

    l_cmd = *((uint32_t*) a_es->buf_in); // Читаем команду

    if (l_cmd >= DRS_PROTO_CMD_MAX){
            a_es->buf_in_size = 0; // Сбрасываем счетчик данных в буфере и считаем его целиком прочитаным
            log_it(L_ERROR, "Unrecognized command 0x%08X", l_cmd);
            return;
    }

    size_t l_cmd_size = sizeof(l_cmd) + g_drs_proto_args_size[l_cmd];
    if( a_es->buf_in_size < l_cmd_size )
        return; // вернёмся сюда тогда, когда клиент вышлет ещё данных

    l_cmd_args = (uint32_t*) (a_es->buf_in + sizeof(uint32_t)); // Тут наши аргументы команды

    // исполняем команду
    drs_proto_cmd( a_es, (drs_proto_cmd_t) l_cmd, l_cmd_args);

    // сдвигаем буфер на l_cmd_size через memmove()
    dap_events_socket_shrink_buf_in(a_es, l_cmd_size );
}

/**
 * @brief Вызывается при отправке данных
 * @param a_es Есокет удаленного клиента
 * @param a_arg не используется
 */
static void s_callback_write (dap_events_socket_t * a_es,void * a_arg )
{
    UNUSED(a_arg);

    // Если нам вообще что-то есть отправить из очереди, то начинаем
    if( DRS_PROTO(a_es)->out_last ){
        drs_proto_data_t * l_out = (drs_proto_data_t*) DRS_PROTO(a_es)->out_last->data;
        assert(l_out);
        switch (l_out->type){
            case DATA_TYPE_MEMORY:
                l_out->data.shift += dap_events_socket_write_unsafe(a_es, l_out->data.ptr+l_out->data.shift,
                                                                       l_out->data.size - l_out->data.shift);
                if (l_out->data.shift >= l_out->data.size ){ // Проверяем, отправили ли мы всё
                    log_it(L_DEBUG, "Have sent succeffuly memory region %zd size", l_out->data.size);
                    s_proto_out_remove(DRS_PROTO(a_es), l_out);
                }
                break;
            case DATA_TYPE_FILE:{
                ssize_t l_res = sendfile( a_es->fd, l_out->file.fd, &l_out->file.offset, BUFSIZ);
                if(l_res == 0 ){
                    log_it(L_DEBUG, "Over file send, %lu bytes were transfered", l_out->file.offset);
                    s_proto_out_remove(DRS_PROTO(a_es), l_out);
                } else if( l_res == EAGAIN || l_res == EWOULDBLOCK ){
                    break;
                } else if (l_res < 0){
                    // неблокирующий сокет не готов сейчас к такому, вернёмся к нему на следующем цикле
                    // все остальные коды ошибок
                    char l_errstr[255];
                    int l_errno = errno;
                    l_errstr[0] = '\0';
                    strerror_r(l_errno, l_errstr, sizeof(l_errstr));
                    log_it(L_ERROR, "Can't send file descriptor: \"%s\" code %d", l_errstr,l_errno );
                    s_proto_out_remove(DRS_PROTO(a_es), l_out);
                }
                break;
            }break;
        }
    }
}

/**
 * @brief s_callback_write_finished
 * @param a_errno
 */
static void s_callback_write_finished(dap_events_socket_t * a_es, void * a_arg, int a_errno)
{
    UNUSED (a_arg);
    log_it(L_DEBUG, "Write finished with errno %d", a_errno);
    if(a_errno == 0) { // это не есть завершение по коду ошибки
        if (DRS_PROTO(a_es)->out_last) // Если нам всё ещё есть что отправить из очереди отправки, то возвращаем флаг готовности к записи
            dap_events_socket_set_writable_unsafe(a_es, true);
    }
}

/**
 * @brief Вызывается при удалении есокета. Удаляет наследуемый объект drs_proto
 * @param a_es Есокет удаленного клиента
 * @param a_arg не используется
 */
static void s_callback_delete (dap_events_socket_t * a_es,void * a_arg )
{
    UNUSED(a_arg);
    // Очищение a_es->_inheritor произойдёт автоматически

    drs_proto_t * l_proto = DRS_PROTO(a_es);
    dap_list_free_full(l_proto->out_first, NULL); // очищает очередь отправки
}

