#ifndef CONTROLFORM_PUSHBTN_H
#define CONTROLFORM_PUSHBTN_H

#include <QWidget>

#include "withrobot_camera.hpp"

namespace Ui {
class ControlFormPushBtn;
}

class ControlFormPushBtn : public QWidget
{
    Q_OBJECT

public:
    explicit ControlFormPushBtn(const char* name, const char* btnName, QWidget *parent = 0);
    ~ControlFormPushBtn();

signals:
    void btnClicked(bool isClicked);

private slots:
    void on_pushButton_clicked();

private:
    Ui::ControlFormPushBtn *ui;
};

#endif // CONTROLFORM_PUSHBTN_H
