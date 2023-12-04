#include <stdlib.h>

typedef void (*FunctionPointer)(const char*, bool, int, float);

struct TopicCommandPair {
    char* topic;
    FunctionPointer command;
};

struct MqttTopicFunctionMap {
    struct TopicCommandPair* entries;
    size_t size;
};


/* functions for Map */
void initializeMap(struct MqttTopicFunctionMap* map, size_t initialSize);
void subscribeTopic(struct MqttTopicFunctionMap* map, const char* topic, FunctionPointer command);
void freeMap(struct MqttTopicFunctionMap* map);
const char** getAllTopics(const struct MqttTopicFunctionMap* map);
const FunctionPointer getFunctionByTopic(struct MqttTopicFunctionMap* map, const char* topic);


