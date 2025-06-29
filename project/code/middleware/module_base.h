#ifndef MODULE_BASE_H
#define MODULE_BASE_H


class Module
{
public:
    Module() = default;
    virtual ~Module() = default;

    virtual void init(){};
    virtual void update(){};
    virtual void reset(){};
};

#endif // MODULE_BASE_H