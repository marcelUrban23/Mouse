#ifndef MOUSEGUI_HPP
#define MOUSEGUI_HPP

#include <vector>
#include <fstream>
#include <complex>

#include <QMainWindow>
#include <QComboBox>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QFile>

#include <QThread>
#include <QMutex>

#include "libmouse.hpp"


class DataSteamer : public QThread
{

Q_OBJECT

bool m_stop_streaming;
Mouse *m_maus;
QMutex m_mutex;
std::vector<unsigned char> m_stream_data;
void run()
{
    m_mutex.lock();
    while(1)
    {
        m_maus->getStreamData(m_stream_data);

        emit sendStreamData(m_stream_data);

        if(m_stop_streaming) break;
    }
    m_mutex.unlock();
    this->deleteLater();
}

public:
DataSteamer(Mouse *maus) :
    m_stop_streaming(false)
{
    m_stream_data.resize(1 << 16);
    m_maus = maus;
}
~DataSteamer(void)
{
    m_stream_data.resize(0);
}
std::atomic_bool m_is_streaming;

signals:
void sendStreamData(std::vector<unsigned char> data);

public slots:

void stopStreaming(void)
{
    std::cerr << "stop" << std::endl;
    m_mutex.unlock();
    m_stop_streaming = true;
}
};



class MouseGUI : public QWidget
{
Q_OBJECT

Mouse m_maus;
DataSteamer *m_streamer;

QTextEdit *m_qte_user_info;
std::vector<std::vector<u_int64_t>> m_filters;
QPushButton *m_qpb_connection,
            *m_qpb_streaming;
QLineEdit *m_qle_center_freq;

QFile *m_file_out;


bool m_streamer_is_running;

public:

MouseGUI(void) : m_streamer_is_running(false)
{
    qRegisterMetaType<std::vector<unsigned char>> ("std::vector<unsigned char>");

    createGUI();

    m_file_out = new QFile("/home/urban/mouse.bin");
    m_file_out->open(QIODevice::WriteOnly);

}

~MouseGUI(void)
{
    m_file_out->flush();
    m_file_out->close();
}


private:

void
createGUI(void)
{
    m_qpb_connection = new QPushButton;
    m_qpb_connection->setText("Verbindung getrennnt");
    connect(m_qpb_connection, SIGNAL(clicked()),
            this            , SLOT(open()));

    QSlider *qs_center_freq = new QSlider(Qt::Horizontal);
    qs_center_freq->setRange(30000, 48000000);
    connect(qs_center_freq, SIGNAL(valueChanged(int)),
            this          , SLOT(setCenterFrequency(int)));

    m_qte_user_info = new QTextEdit;

    m_qpb_streaming = new QPushButton;
    m_qpb_streaming->setText("Stream ist aus");
    connect(m_qpb_streaming, SIGNAL(clicked()),
            this        , SLOT(streamOn()));

    m_qle_center_freq = new QLineEdit;
    m_qle_center_freq->setMaximumWidth(100);

    QHBoxLayout *qhbl_freq = new QHBoxLayout;
    qhbl_freq->addWidget(m_qpb_streaming);
    qhbl_freq->addWidget(new QLabel("Frequenz: "));
    qhbl_freq->addWidget(m_qle_center_freq);
    qhbl_freq->addWidget(qs_center_freq);

    QVBoxLayout *qvbl_main = new QVBoxLayout;
    qvbl_main->addWidget(m_qpb_connection);
    qvbl_main->addLayout(qhbl_freq);
//    qvbl_main->addWidget(m_waterfall);
    qvbl_main->addWidget(m_qte_user_info);

    setLayout(qvbl_main);
}

private slots:

void
open(void)
{
    if(m_maus.m_is_open)
    {
        m_filters.resize(0);
        m_maus.close();
        m_qpb_connection->setText("Verbindung getrennt");
        return;
    }
    if(m_maus.open())
    {
        m_qte_user_info->append(QString::fromStdString(m_maus.getError()));
        return;
    }
    m_qpb_connection->setText("Verbindung ergestellt");

    m_filters = m_maus.getFilter();
    if(m_filters.empty())
    {
        m_qte_user_info->append(QString::fromStdString(m_maus.getError()));
    }

    m_maus.setGPIFMode();
}

void
streamOn(void)
{
    if(m_streamer_is_running)
    {
        m_streamer->disconnect();
        m_qpb_streaming->setText("Stream ist aus");
        m_streamer_is_running = false;
        return;
    }
    m_qpb_streaming->setText("Stream ist ein");
    m_streamer = new DataSteamer(&m_maus);
    connect(m_qpb_streaming, SIGNAL(clicked()),
            m_streamer     , SLOT(stopStreaming()));
    connect(m_streamer, SIGNAL(sendStreamData(std::vector<unsigned char>)),
            this      , SLOT(feedWaterfall(std::vector<unsigned char>)));
    m_streamer_is_running = true;
    m_streamer->start();
}

public slots:

void
feedWaterfall(std::vector<unsigned char> data)
{

    m_file_out->write(reinterpret_cast<char*>(data.data()), data.size());


    std::vector<short> input(reinterpret_cast<short*>(data.data()),
                             reinterpret_cast<short*>(data.data()) + data.size() / 2);

    std::vector<std::complex<float>> output(input.size());
    std::transform(input.begin(), input.end(),
                   output.begin(),
                   [](std::complex<short> value) -> std::complex<float>
                   {return std::complex<float>(
                                static_cast<float>(value.real(),
                                static_cast<float>(value.imag())));});
}

void setCenterFrequency(int frequency)
{
    m_qle_center_freq->setText(QString::number(frequency));
    if( ! m_maus.m_is_open) return;
    m_maus.setCenterFrequency(static_cast<int32_t>(frequency));
}


};

#endif // MOUSEGUI_HPP
