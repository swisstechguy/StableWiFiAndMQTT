#include "MqttTopicFunctionMap.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>




/*
functions for Mqtt Map
*/

void initializeMap(struct MqttTopicFunctionMap* map, size_t initialSize) {
    map->entries = (struct TopicCommandPair*)malloc(initialSize * sizeof(struct TopicCommandPair));
    map->size = initialSize;
}

void subscribeTopic(struct MqttTopicFunctionMap* map, const char* topic, FunctionPointer command) {
    if (map->size == 0) {
        map->entries = (struct TopicCommandPair*)malloc(sizeof(struct TopicCommandPair));
        map->size = 1;
    } else if (map->size > 0) {
        map->entries = (struct TopicCommandPair*)realloc(map->entries, (map->size + 1) * sizeof(struct TopicCommandPair));
        map->size++;
    }

    map->entries[map->size - 1].topic = strdup(topic);
    map->entries[map->size - 1].command = command;
}

const char** getAllTopics(const struct MqttTopicFunctionMap* map) {
    const char** topics = (const char**)malloc(map->size * sizeof(const char*));
    for (size_t i = 0; i < map->size; ++i) {
        topics[i] = map->entries[i].topic;
    }
    return topics;
}

const FunctionPointer getFunctionByTopic(struct MqttTopicFunctionMap* map, const char* topic) {
    for (size_t i = 0; i < map->size; ++i) {
        if (strcmp(map->entries[i].topic, topic) == 0) {
            return map->entries[i].command;
        }
    }

    return NULL;
}

void freeMap(struct MqttTopicFunctionMap* map) {
    for (size_t i = 0; i < map->size; ++i) {
        free(map->entries[i].topic);
    }
    free(map->entries);
    map->size = 0;
}



