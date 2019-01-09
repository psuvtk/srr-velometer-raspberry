#include "./include/bytearray.h"

void bytearray_init(bytearray_t *buf) {
#ifdef DEBUG
    printf("enter func: bytearray_init\n ");
#endif
    buf->data = NULL;
    buf->size = 0;
    buf->capacity = 0;

#ifdef DEBUG
    printf("exit func: bytearray_init\n ");
#endif
}

void bytearray_destroy(bytearray_t *buf) {
#ifdef DEBUG
    printf("enter func: bytearray_destroy\n ");
#endif

    if (buf->data != NULL)
        free(buf->data);
    buf->data = NULL;
    buf->size = 0;
    buf->capacity = 0;

#ifdef DEBUG
    printf("exit func: bytearray_destroy\n ");
#endif
}

void bytearray_append(bytearray_t *buf, char *pdata, int len) {
#ifdef DEBUG
    printf("enter func: bytearray_append\n ");
#endif

    if (buf->data == NULL || buf->capacity == 0) {
        buf->data = (char *)malloc(DEFAULT_BUF_LEN * sizeof(char));
        if (buf->data == NULL) {
            perror("malloc failed.");
            exit(-1);
        }

        memcpy(buf->data, pdata, len);
        buf->size = len;
        buf->capacity = DEFAULT_BUF_LEN;
        return;
    }

    if (buf->capacity - buf->size >= len) {
        // 足以容纳
        memcpy(&(buf->data)[buf->size], pdata, len);
        buf->size += len;
        return;
    } else {
        // 不足以容纳
        while (buf->capacity < buf->size+len)
            buf->capacity *= 2;

        char *new_buf = (char *)malloc(buf->capacity * sizeof(char));
        if (new_buf == NULL) {
            perror("malloc new buffer faild");
            exit(-1);
        }

        memcpy(new_buf, buf->data, buf->size);
        memcpy(&new_buf[buf->size], pdata, len);
        buf->data = new_buf;
        buf->size += len;
        free(buf->data);
    }

#ifdef DEBUG
    printf("exit func: bytearray_append\n ");
#endif

}

void bytearray_lremove(bytearray_t *buf, int len) {
#ifdef DEBUG
    printf("enter func: bytearray_lremove\n ");
#endif
    buf->size -= len;
    memcpy(buf->data, &(buf->data)[len], buf->size);

#ifdef DEBUG
    printf("exit func: bytearray_append\n ");
#endif

}

void bytearray_clear(bytearray_t *buf) {
    buf->size = 0;
}

bool bytearray_startswith(const bytearray_t *buf, const char *pdata, int len) {
#ifdef DEBUG
    printf("enter func: bytearray_startwith\n ");
#endif

    if (buf->size < len)
        return false;
    return memcmp(buf->data, pdata, len) == 0;
}

int bytearray_find(const bytearray_t *buf, const char *pdata, int len, int skip) {
#ifdef DEBUG
    printf("enter func: bytearray_find\n ");
#endif

    if (buf->size < len+skip) return -1;
    // TODO: 是否有更高效匹配的实现？
    for (int i = skip; i < buf->size-len; i++) {
        if (memcmp(&(buf->data)[i], pdata, len) == 0) {
#ifdef DEBUG
    printf("enter func: bytearray_find(inner#%d)\n ", i);
#endif
            return i;
        }
    }
#ifdef DEBUG
    printf("enter func: bytearray_find(outer)\n ");
#endif
    return -1;
}