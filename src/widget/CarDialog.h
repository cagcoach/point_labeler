#ifndef CARDIALOG_H_
#define CARDIALOG_H_

#include <QDialog>
#include "AutoAuto.h"
#include <QListWidget> 
#include <QDialogButtonBox> 
#include <QVBoxLayout> 

class CarDialog : public QDialog
{
	Q_OBJECT

public:
	CarDialog(std::shared_ptr<AutoAuto> a, QWidget* parent = 0, bool more=true);
	~CarDialog();

signals:
	void changeCar(std::shared_ptr<AutoAuto>, int);
	void saveCar(std::shared_ptr<AutoAuto>);
	void continueCar(std::shared_ptr<AutoAuto>);
	void windowClosed();
private slots:
	void save();
	void discard();
	void continue_();
	void selectionChanged();
public slots:
	void viewChanged();
private:
	QListWidget *carList;
	QDialogButtonBox *buttonBox;
	std::shared_ptr<AutoAuto> a;
	QVBoxLayout *mainLayout;
	void filllist();
	void closeEvent(QCloseEvent *event);
	void disconn();
};

#endif