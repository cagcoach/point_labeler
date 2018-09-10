#ifndef CARDIALOG_H_
#define CARDIALOG_H_

#include <QDialog>
#include "AutoAuto.h"

class CarDialog : public QDialog
{
	Q_OBJECT

public:
	CarDialog(std::shared_ptr<AutoAuto> a);

signals:
	void changeCar(std::shared_ptr<AutoAuto>, int);
private slots:
	void save();
	void discard();
	void selectionChanged();
private:
	QListWidget *carList;
	QDialogButtonBox *buttonBox;
	std::shared_ptr<AutoAuto> a;
	filllist();
}

#endif