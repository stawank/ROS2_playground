// C++ Program to implement a queue using array
#ifndef TURTLE_QUEUE_HPP
#define TURTLE_QUEUE_HPP

// Defining Pose of a turtle
struct TurtlePose{
    float x;
    float y;
    float theta;
};

// defining the max size of the queue
#define MAX_SIZE 100

// Implement the queue data structure
class Queue {
public:
    int front;
    int rear;
    TurtlePose arr[MAX_SIZE];

    // initializing pointers in the constructor
    Queue(): front(-1), rear(-1) {}

    // Function to check if the queue is empty or not
    bool isEmpty() { return front == -1 || front > rear; }

    // Function to check if the queue is full or not
    bool isFull() { return rear == MAX_SIZE - 1; }

    // Function to get the front element of the queue
    TurtlePose getFront()
    {
        if (isEmpty()) {
            TurtlePose origin;
            origin.x =0;
            origin.y = 0;
            origin.theta = 0;
            return origin;
        }
        return arr[front];
    }

    // Function to get the rear element of the queue
    TurtlePose getRear()
    {
        if (isEmpty()) {
            TurtlePose origin;
            origin.x =0;
            origin.y = 0;
            origin.theta = 0;
            return origin;
        }
        return arr[rear];
    }

    // Function to enqueue elements from the queue
    void enqueue(TurtlePose val)
    {
        // Check overflow condition
        if (isFull()) {
            
            return;
        }
        // if queue is empty, set front to 0
        if (isEmpty())
            front = 0;

        rear++;
        arr[rear] = val;
    }

    // Function to dequeue elements from the queue
    TurtlePose dequeue()
    {
        // Check underflow condition
        if (isEmpty()) {
            TurtlePose origin;
            origin.x =0;
            origin.y = 0;
            origin.theta = 0;
            return origin;
        }
        TurtlePose ans = arr[front];
        front++;

        // if queue becomes empty, reset both pointers
        if (isEmpty())
            front = rear = -1;

        return ans;
    }

};

#endif