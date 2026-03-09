#include <Arduino.h>
struct JobNode
{
    float total_len;
    float front_len;
    float back_len;
    int quantity;
    JobNode *next;

    JobNode(float t, float f, float b, int q)
    {
        total_len = t;
        front_len = f;
        back_len = b;
        quantity = q;
        next = nullptr;
    }
};

class JobQueue
{
private:
    JobNode *head;
    JobNode *tail;
    int count;

public:
    JobQueue()
    {
        head = nullptr;
        tail = nullptr;
        count = 0;
    }

    //เพิ่มคิวงานใหม่(ต่อท้าย)
    void push_back(float t, float f, float b, int q)
    {
        JobNode *new_node = new JobNode(t, f, b, q);

        if (head == nullptr)
        {
            // ถ้าคิวว่าง ให้หัวและหางชี้ที่เดียวกัน
            head = new_node;
            tail = new_node;
        }
        else
        {
            // ถ้ามีคิวอยู่แล้ว เอาไปต่อท้ายหางขบวน แล้วย้ายหาง
            tail->next = new_node;
            tail = new_node;
        }
        count++;
    }

    //ดึงงานแรกสุดออกไปทำ
    //คืนค่าเป็น true ถ้ามีคิวให้ดึง, คืนค่า false ถ้าคิวว่าง
    bool pop_front(float &t, float &f, float &b, int &q)
    {
        if (head == nullptr)
            return false;

        JobNode *target = head;
        t = target->total_len;
        f = target->front_len;
        b = target->back_len;
        q = target->quantity;

        head = head->next;

        if (head == nullptr)
        {
            tail = nullptr;
        }

        delete target;
        count--;
        return true;
    }

    bool remove_at(int index)//ลบคิวที่ตำแหน่งใดๆ
    {
        if (head == nullptr || index < 0 || index >= count)
            return false;

        if (index == 0)
        {
            float dummyT, dummyF, dummyB;
            int dummyQ;
            return pop_front(dummyT, dummyF, dummyB, dummyQ); // ใช้ pop_front แทนไปเลย
        }

        JobNode *p = head;
        for (int i = 0; i < index - 1; i++)
        {
            p = p->next;
        }

        JobNode *target = p->next;
        p->next = target->next;

        if (target == tail)
        {
            tail = p; // ถ้าลบตัวสุดท้าย ต้องถอยหางกลับมา
        }

        delete target;
        count--;
        return true;
    }

    void clear()
    { // ล้างคิวทั้งหมด
        while (head != nullptr)
        {
            JobNode *temp = head;
            head = head->next;
            delete temp;
        }
        tail = nullptr;
        count = 0;
    }

    int get_count() { return count; }
    bool is_empty() { return head == nullptr; }
};