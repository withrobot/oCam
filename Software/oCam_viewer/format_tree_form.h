#ifndef FORMAT_TREE_FORM_H
#define FORMAT_TREE_FORM_H

#include <QWidget>
#include <QTreeWidget>

namespace Ui {
class FormatTreeForm;
}

class FormatTreeForm : public QWidget
{
    Q_OBJECT

public:
    explicit FormatTreeForm(QString name, QWidget *parent = 0);
    ~FormatTreeForm();

    void add_format_list(const char* format_name, std::vector<const char *>& frame_rate_list);

    inline bool is_changed() const { return changed; }

    std::string get_format_name();

public slots:
    void change_title(std::string str);

private slots:
    void on_btnApply_clicked();

private:
    Ui::FormatTreeForm *ui;

    bool changed;

    void add_format_tree(QString format_name, std::vector<const char *>& frame_rate_list);
    void add_rate_tree(QTreeWidgetItem* parent, QString frame_rate_name);
};

#endif // FORMAT_TREE_FORM_H
