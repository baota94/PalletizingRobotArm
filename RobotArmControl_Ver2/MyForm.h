#pragma once
#include <cstdlib>
#include <string>
#include <array>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <math.h>
#include "CmdCode.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <numbers>
#define M_PI 3.14159265358979323846
using namespace cv;
namespace RobotArmControlVer2 {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO::Ports;
	using namespace System::Threading;
	using namespace System::Net::Sockets;
	using namespace System::Net;
	using namespace System::Threading::Tasks;
	
	/// <summary>
	/// Summary for MyForm
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::GroupBox^ groupBox1;
	private: System::Windows::Forms::ComboBox^ cbxConnection;
	private: System::Windows::Forms::Button^ btn_Connect;


	private: System::Windows::Forms::PictureBox^ pictureBox1;
	private: System::Windows::Forms::GroupBox^ groupBox2;
	private: System::Windows::Forms::GroupBox^ grpControlPanel;
	private: System::Windows::Forms::GroupBox^ groupBox3;
	private: System::Windows::Forms::GroupBox^ groupBox5;
	private: System::Windows::Forms::Button^ btnRunRobot;
	private: System::Windows::Forms::Button^ btnFlash;


	private: System::Windows::Forms::GroupBox^ groupBox4;
	private: System::Windows::Forms::Button^ btnSavePath;
	private: System::Windows::Forms::Button^ btnClearPath;


	private: System::Windows::Forms::Button^ btnClearPos;

	private: System::Windows::Forms::Button^ btnSavePos;

	private: System::Windows::Forms::Button^ btnModeProcess;

	private: System::Windows::Forms::ComboBox^ cbxMode;

	private: System::Windows::Forms::GroupBox^ grpFwd;

	private: System::Windows::Forms::GroupBox^ groupBox8;
	private: System::Windows::Forms::TextBox^ nbxTheta4;

	private: System::Windows::Forms::TrackBar^ trkTheta4;

	private: System::Windows::Forms::GroupBox^ groupBox10;
	private: System::Windows::Forms::TextBox^ nbxTheta5;

	private: System::Windows::Forms::TrackBar^ trkTheta5;

	private: System::Windows::Forms::GroupBox^ groupBox12;
	private: System::Windows::Forms::TextBox^ nbxTheta6;

	private: System::Windows::Forms::TrackBar^ trkTheta6;

	private: System::Windows::Forms::GroupBox^ groupBox11;
	private: System::Windows::Forms::TextBox^ nbxTheta3;

	private: System::Windows::Forms::TrackBar^ trkTheta3;

	private: System::Windows::Forms::GroupBox^ groupBox9;
	private: System::Windows::Forms::TextBox^ nbxTheta2;

	private: System::Windows::Forms::TrackBar^ trkTheta2;

	private: System::Windows::Forms::GroupBox^ groupBox7;
	private: System::Windows::Forms::TextBox^ nbxTheta1;

	private: System::Windows::Forms::TrackBar^ trkTheta1;

	private: System::Windows::Forms::GroupBox^ grpInv;

	private: System::Windows::Forms::GroupBox^ groupBox14;
	private: System::Windows::Forms::TextBox^ nbxPosRoll;
	private: System::Windows::Forms::TrackBar^ trkPosRoll;


	private: System::Windows::Forms::GroupBox^ groupBox15;
	private: System::Windows::Forms::TextBox^ nbxPosPitch;
	private: System::Windows::Forms::TrackBar^ trkPosPitch;


	private: System::Windows::Forms::GroupBox^ groupBox16;
	private: System::Windows::Forms::TextBox^ nbxPosGrip;
	private: System::Windows::Forms::TrackBar^ trkPosGrip;


	private: System::Windows::Forms::GroupBox^ groupBox17;
	private: System::Windows::Forms::TextBox^ nbxPosZ;

	private: System::Windows::Forms::TrackBar^ trkPosZ;

	private: System::Windows::Forms::GroupBox^ groupBox18;
	private: System::Windows::Forms::TextBox^ nbxPosY;

	private: System::Windows::Forms::TrackBar^ trkPosY;

	private: System::Windows::Forms::GroupBox^ groupBox19;
	private: System::Windows::Forms::TextBox^ nbxPosX;

	private: System::Windows::Forms::TrackBar^ trkPosX;



	private: System::Windows::Forms::Timer^ timer1;
private: System::Windows::Forms::GroupBox^ groupBox6;
private: System::Windows::Forms::RichTextBox^ richTextBox1;
private: System::Windows::Forms::GroupBox^ groupBox13;
private: System::Windows::Forms::RadioButton^ radioButton3;
private: System::Windows::Forms::RadioButton^ radioButton2;
private: System::Windows::Forms::RadioButton^ radioButton1;
	private: System::ComponentModel::IContainer^ components;


	protected:

	protected:

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			System::ComponentModel::ComponentResourceManager^ resources = (gcnew System::ComponentModel::ComponentResourceManager(MyForm::typeid));
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->btn_Connect = (gcnew System::Windows::Forms::Button());
			this->cbxConnection = (gcnew System::Windows::Forms::ComboBox());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
			this->grpControlPanel = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox5 = (gcnew System::Windows::Forms::GroupBox());
			this->btnRunRobot = (gcnew System::Windows::Forms::Button());
			this->btnFlash = (gcnew System::Windows::Forms::Button());
			this->groupBox4 = (gcnew System::Windows::Forms::GroupBox());
			this->btnSavePath = (gcnew System::Windows::Forms::Button());
			this->btnClearPath = (gcnew System::Windows::Forms::Button());
			this->btnClearPos = (gcnew System::Windows::Forms::Button());
			this->btnSavePos = (gcnew System::Windows::Forms::Button());
			this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
			this->btnModeProcess = (gcnew System::Windows::Forms::Button());
			this->cbxMode = (gcnew System::Windows::Forms::ComboBox());
			this->grpFwd = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox8 = (gcnew System::Windows::Forms::GroupBox());
			this->nbxTheta4 = (gcnew System::Windows::Forms::TextBox());
			this->trkTheta4 = (gcnew System::Windows::Forms::TrackBar());
			this->groupBox10 = (gcnew System::Windows::Forms::GroupBox());
			this->nbxTheta5 = (gcnew System::Windows::Forms::TextBox());
			this->trkTheta5 = (gcnew System::Windows::Forms::TrackBar());
			this->groupBox12 = (gcnew System::Windows::Forms::GroupBox());
			this->nbxTheta6 = (gcnew System::Windows::Forms::TextBox());
			this->trkTheta6 = (gcnew System::Windows::Forms::TrackBar());
			this->groupBox11 = (gcnew System::Windows::Forms::GroupBox());
			this->nbxTheta3 = (gcnew System::Windows::Forms::TextBox());
			this->trkTheta3 = (gcnew System::Windows::Forms::TrackBar());
			this->groupBox9 = (gcnew System::Windows::Forms::GroupBox());
			this->nbxTheta2 = (gcnew System::Windows::Forms::TextBox());
			this->trkTheta2 = (gcnew System::Windows::Forms::TrackBar());
			this->groupBox7 = (gcnew System::Windows::Forms::GroupBox());
			this->nbxTheta1 = (gcnew System::Windows::Forms::TextBox());
			this->trkTheta1 = (gcnew System::Windows::Forms::TrackBar());
			this->grpInv = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox14 = (gcnew System::Windows::Forms::GroupBox());
			this->nbxPosRoll = (gcnew System::Windows::Forms::TextBox());
			this->trkPosRoll = (gcnew System::Windows::Forms::TrackBar());
			this->groupBox15 = (gcnew System::Windows::Forms::GroupBox());
			this->nbxPosPitch = (gcnew System::Windows::Forms::TextBox());
			this->trkPosPitch = (gcnew System::Windows::Forms::TrackBar());
			this->groupBox16 = (gcnew System::Windows::Forms::GroupBox());
			this->nbxPosGrip = (gcnew System::Windows::Forms::TextBox());
			this->trkPosGrip = (gcnew System::Windows::Forms::TrackBar());
			this->groupBox17 = (gcnew System::Windows::Forms::GroupBox());
			this->nbxPosZ = (gcnew System::Windows::Forms::TextBox());
			this->trkPosZ = (gcnew System::Windows::Forms::TrackBar());
			this->groupBox18 = (gcnew System::Windows::Forms::GroupBox());
			this->nbxPosY = (gcnew System::Windows::Forms::TextBox());
			this->trkPosY = (gcnew System::Windows::Forms::TrackBar());
			this->groupBox19 = (gcnew System::Windows::Forms::GroupBox());
			this->nbxPosX = (gcnew System::Windows::Forms::TextBox());
			this->trkPosX = (gcnew System::Windows::Forms::TrackBar());
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->groupBox6 = (gcnew System::Windows::Forms::GroupBox());
			this->richTextBox1 = (gcnew System::Windows::Forms::RichTextBox());
			this->groupBox13 = (gcnew System::Windows::Forms::GroupBox());
			this->radioButton3 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButton2 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButton1 = (gcnew System::Windows::Forms::RadioButton());
			this->groupBox1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			this->groupBox2->SuspendLayout();
			this->grpControlPanel->SuspendLayout();
			this->groupBox5->SuspendLayout();
			this->groupBox4->SuspendLayout();
			this->groupBox3->SuspendLayout();
			this->grpFwd->SuspendLayout();
			this->groupBox8->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkTheta4))->BeginInit();
			this->groupBox10->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkTheta5))->BeginInit();
			this->groupBox12->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkTheta6))->BeginInit();
			this->groupBox11->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkTheta3))->BeginInit();
			this->groupBox9->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkTheta2))->BeginInit();
			this->groupBox7->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkTheta1))->BeginInit();
			this->grpInv->SuspendLayout();
			this->groupBox14->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkPosRoll))->BeginInit();
			this->groupBox15->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkPosPitch))->BeginInit();
			this->groupBox16->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkPosGrip))->BeginInit();
			this->groupBox17->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkPosZ))->BeginInit();
			this->groupBox18->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkPosY))->BeginInit();
			this->groupBox19->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkPosX))->BeginInit();
			this->groupBox6->SuspendLayout();
			this->groupBox13->SuspendLayout();
			this->SuspendLayout();
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->btn_Connect);
			this->groupBox1->Controls->Add(this->cbxConnection);
			this->groupBox1->Location = System::Drawing::Point(12, 12);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(256, 56);
			this->groupBox1->TabIndex = 0;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Connect Panel";
			// 
			// btn_Connect
			// 
			this->btn_Connect->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->btn_Connect->ForeColor = System::Drawing::Color::Green;
			this->btn_Connect->Location = System::Drawing::Point(164, 19);
			this->btn_Connect->Name = L"btn_Connect";
			this->btn_Connect->Size = System::Drawing::Size(75, 23);
			this->btn_Connect->TabIndex = 2;
			this->btn_Connect->Text = L"Connect";
			this->btn_Connect->UseVisualStyleBackColor = true;
			this->btn_Connect->Click += gcnew System::EventHandler(this, &MyForm::btn_Connect_Click);
			// 
			// cbxConnection
			// 
			this->cbxConnection->FormattingEnabled = true;
			this->cbxConnection->Location = System::Drawing::Point(6, 19);
			this->cbxConnection->Name = L"cbxConnection";
			this->cbxConnection->Size = System::Drawing::Size(152, 21);
			this->cbxConnection->TabIndex = 1;
			this->cbxConnection->SelectedIndexChanged += gcnew System::EventHandler(this, &MyForm::cbxConnection_SelectedIndexChanged);
			// 
			// pictureBox1
			// 
			this->pictureBox1->Location = System::Drawing::Point(8, 19);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(240, 240);
			this->pictureBox1->TabIndex = 1;
			this->pictureBox1->TabStop = false;
			// 
			// groupBox2
			// 
			this->groupBox2->Controls->Add(this->pictureBox1);
			this->groupBox2->Location = System::Drawing::Point(12, 74);
			this->groupBox2->Name = L"groupBox2";
			this->groupBox2->Size = System::Drawing::Size(256, 270);
			this->groupBox2->TabIndex = 2;
			this->groupBox2->TabStop = false;
			this->groupBox2->Text = L"Camera";
			// 
			// grpControlPanel
			// 
			this->grpControlPanel->Controls->Add(this->groupBox5);
			this->grpControlPanel->Controls->Add(this->groupBox4);
			this->grpControlPanel->Controls->Add(this->groupBox3);
			this->grpControlPanel->Enabled = false;
			this->grpControlPanel->Location = System::Drawing::Point(12, 350);
			this->grpControlPanel->Name = L"grpControlPanel";
			this->grpControlPanel->Size = System::Drawing::Size(256, 280);
			this->grpControlPanel->TabIndex = 3;
			this->grpControlPanel->TabStop = false;
			this->grpControlPanel->Text = L"Control Panel";
			// 
			// groupBox5
			// 
			this->groupBox5->Controls->Add(this->btnRunRobot);
			this->groupBox5->Controls->Add(this->btnFlash);
			this->groupBox5->Location = System::Drawing::Point(6, 175);
			this->groupBox5->Name = L"groupBox5";
			this->groupBox5->Size = System::Drawing::Size(242, 100);
			this->groupBox5->TabIndex = 4;
			this->groupBox5->TabStop = false;
			this->groupBox5->Text = L"Robot Control";
			// 
			// btnRunRobot
			// 
			this->btnRunRobot->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->btnRunRobot->ForeColor = System::Drawing::Color::Green;
			this->btnRunRobot->Location = System::Drawing::Point(75, 59);
			this->btnRunRobot->Name = L"btnRunRobot";
			this->btnRunRobot->Size = System::Drawing::Size(106, 35);
			this->btnRunRobot->TabIndex = 4;
			this->btnRunRobot->Text = L"▶️ Start";
			this->btnRunRobot->UseVisualStyleBackColor = true;
			this->btnRunRobot->Click += gcnew System::EventHandler(this, &MyForm::btnRunRobot_Click);
			// 
			// btnFlash
			// 
			this->btnFlash->Location = System::Drawing::Point(91, 19);
			this->btnFlash->Name = L"btnFlash";
			this->btnFlash->Size = System::Drawing::Size(75, 23);
			this->btnFlash->TabIndex = 4;
			this->btnFlash->Text = L"Flash On";
			this->btnFlash->UseVisualStyleBackColor = true;
			this->btnFlash->Click += gcnew System::EventHandler(this, &MyForm::btnFlash_Click);
			// 
			// groupBox4
			// 
			this->groupBox4->Controls->Add(this->btnSavePath);
			this->groupBox4->Controls->Add(this->btnClearPath);
			this->groupBox4->Controls->Add(this->btnClearPos);
			this->groupBox4->Controls->Add(this->btnSavePos);
			this->groupBox4->Enabled = false;
			this->groupBox4->Location = System::Drawing::Point(6, 69);
			this->groupBox4->Name = L"groupBox4";
			this->groupBox4->Size = System::Drawing::Size(242, 100);
			this->groupBox4->TabIndex = 4;
			this->groupBox4->TabStop = false;
			this->groupBox4->Text = L"Robot Teaching";
			// 
			// btnSavePath
			// 
			this->btnSavePath->Location = System::Drawing::Point(22, 58);
			this->btnSavePath->Name = L"btnSavePath";
			this->btnSavePath->Size = System::Drawing::Size(75, 23);
			this->btnSavePath->TabIndex = 4;
			this->btnSavePath->Text = L"Save Path";
			this->btnSavePath->UseVisualStyleBackColor = true;
			// 
			// btnClearPath
			// 
			this->btnClearPath->Location = System::Drawing::Point(134, 58);
			this->btnClearPath->Name = L"btnClearPath";
			this->btnClearPath->Size = System::Drawing::Size(75, 23);
			this->btnClearPath->TabIndex = 4;
			this->btnClearPath->Text = L"Clear Path";
			this->btnClearPath->UseVisualStyleBackColor = true;
			this->btnClearPath->Click += gcnew System::EventHandler(this, &MyForm::btnClearPath_Click);
			// 
			// btnClearPos
			// 
			this->btnClearPos->Location = System::Drawing::Point(134, 19);
			this->btnClearPos->Name = L"btnClearPos";
			this->btnClearPos->Size = System::Drawing::Size(75, 23);
			this->btnClearPos->TabIndex = 4;
			this->btnClearPos->Text = L"Clear Pos";
			this->btnClearPos->UseVisualStyleBackColor = true;
			this->btnClearPos->Click += gcnew System::EventHandler(this, &MyForm::btnClearPos_Click);
			// 
			// btnSavePos
			// 
			this->btnSavePos->Location = System::Drawing::Point(22, 19);
			this->btnSavePos->Name = L"btnSavePos";
			this->btnSavePos->Size = System::Drawing::Size(75, 23);
			this->btnSavePos->TabIndex = 4;
			this->btnSavePos->Text = L"Save Pos";
			this->btnSavePos->UseVisualStyleBackColor = true;
			this->btnSavePos->Click += gcnew System::EventHandler(this, &MyForm::btnSavePos_Click);
			// 
			// groupBox3
			// 
			this->groupBox3->Controls->Add(this->btnModeProcess);
			this->groupBox3->Controls->Add(this->cbxMode);
			this->groupBox3->Location = System::Drawing::Point(6, 19);
			this->groupBox3->Name = L"groupBox3";
			this->groupBox3->Size = System::Drawing::Size(242, 44);
			this->groupBox3->TabIndex = 4;
			this->groupBox3->TabStop = false;
			this->groupBox3->Text = L"Mode";
			// 
			// btnModeProcess
			// 
			this->btnModeProcess->Location = System::Drawing::Point(158, 14);
			this->btnModeProcess->Name = L"btnModeProcess";
			this->btnModeProcess->Size = System::Drawing::Size(75, 23);
			this->btnModeProcess->TabIndex = 1;
			this->btnModeProcess->Text = L"Process";
			this->btnModeProcess->UseVisualStyleBackColor = true;
			this->btnModeProcess->Click += gcnew System::EventHandler(this, &MyForm::btnModeProcess_Click);
			// 
			// cbxMode
			// 
			this->cbxMode->DropDownStyle = System::Windows::Forms::ComboBoxStyle::DropDownList;
			this->cbxMode->FormattingEnabled = true;
			this->cbxMode->Items->AddRange(gcnew cli::array< System::Object^  >(2) { L"Teach Mode", L"Run Robot" });
			this->cbxMode->Location = System::Drawing::Point(3, 16);
			this->cbxMode->Name = L"cbxMode";
			this->cbxMode->Size = System::Drawing::Size(149, 21);
			this->cbxMode->TabIndex = 1;
			// 
			// grpFwd
			// 
			this->grpFwd->Controls->Add(this->groupBox8);
			this->grpFwd->Controls->Add(this->groupBox10);
			this->grpFwd->Controls->Add(this->groupBox12);
			this->grpFwd->Controls->Add(this->groupBox11);
			this->grpFwd->Controls->Add(this->groupBox9);
			this->grpFwd->Controls->Add(this->groupBox7);
			this->grpFwd->Enabled = false;
			this->grpFwd->Location = System::Drawing::Point(274, 163);
			this->grpFwd->Name = L"grpFwd";
			this->grpFwd->Size = System::Drawing::Size(423, 233);
			this->grpFwd->TabIndex = 4;
			this->grpFwd->TabStop = false;
			this->grpFwd->Text = L"Forward Kinematic";
			// 
			// groupBox8
			// 
			this->groupBox8->Controls->Add(this->nbxTheta4);
			this->groupBox8->Controls->Add(this->trkTheta4);
			this->groupBox8->Location = System::Drawing::Point(217, 19);
			this->groupBox8->Name = L"groupBox8";
			this->groupBox8->Size = System::Drawing::Size(200, 64);
			this->groupBox8->TabIndex = 0;
			this->groupBox8->TabStop = false;
			this->groupBox8->Text = L"Theta 4";
			// 
			// nbxTheta4
			// 
			this->nbxTheta4->Cursor = System::Windows::Forms::Cursors::Default;
			this->nbxTheta4->Location = System::Drawing::Point(138, 19);
			this->nbxTheta4->Name = L"nbxTheta4";
			this->nbxTheta4->ReadOnly = true;
			this->nbxTheta4->Size = System::Drawing::Size(56, 20);
			this->nbxTheta4->TabIndex = 5;
			this->nbxTheta4->Text = L"90.0";
			this->nbxTheta4->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// trkTheta4
			// 
			this->trkTheta4->Location = System::Drawing::Point(6, 19);
			this->trkTheta4->Maximum = 1800;
			this->trkTheta4->Name = L"trkTheta4";
			this->trkTheta4->Size = System::Drawing::Size(126, 45);
			this->trkTheta4->TabIndex = 1;
			this->trkTheta4->Value = 900;
			this->trkTheta4->Scroll += gcnew System::EventHandler(this, &MyForm::trkTheta4_Scroll);
			// 
			// groupBox10
			// 
			this->groupBox10->Controls->Add(this->nbxTheta5);
			this->groupBox10->Controls->Add(this->trkTheta5);
			this->groupBox10->Location = System::Drawing::Point(217, 89);
			this->groupBox10->Name = L"groupBox10";
			this->groupBox10->Size = System::Drawing::Size(200, 64);
			this->groupBox10->TabIndex = 0;
			this->groupBox10->TabStop = false;
			this->groupBox10->Text = L"Theta 5";
			// 
			// nbxTheta5
			// 
			this->nbxTheta5->Cursor = System::Windows::Forms::Cursors::Default;
			this->nbxTheta5->Location = System::Drawing::Point(138, 19);
			this->nbxTheta5->Name = L"nbxTheta5";
			this->nbxTheta5->ReadOnly = true;
			this->nbxTheta5->Size = System::Drawing::Size(56, 20);
			this->nbxTheta5->TabIndex = 5;
			this->nbxTheta5->Text = L"90.0";
			this->nbxTheta5->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// trkTheta5
			// 
			this->trkTheta5->Location = System::Drawing::Point(6, 19);
			this->trkTheta5->Maximum = 1800;
			this->trkTheta5->Name = L"trkTheta5";
			this->trkTheta5->Size = System::Drawing::Size(126, 45);
			this->trkTheta5->TabIndex = 1;
			this->trkTheta5->Value = 900;
			this->trkTheta5->Scroll += gcnew System::EventHandler(this, &MyForm::trkTheta5_Scroll);
			// 
			// groupBox12
			// 
			this->groupBox12->Controls->Add(this->nbxTheta6);
			this->groupBox12->Controls->Add(this->trkTheta6);
			this->groupBox12->Location = System::Drawing::Point(217, 159);
			this->groupBox12->Name = L"groupBox12";
			this->groupBox12->Size = System::Drawing::Size(200, 64);
			this->groupBox12->TabIndex = 0;
			this->groupBox12->TabStop = false;
			this->groupBox12->Text = L"Theta 6";
			// 
			// nbxTheta6
			// 
			this->nbxTheta6->Cursor = System::Windows::Forms::Cursors::Default;
			this->nbxTheta6->Location = System::Drawing::Point(138, 19);
			this->nbxTheta6->Name = L"nbxTheta6";
			this->nbxTheta6->ReadOnly = true;
			this->nbxTheta6->Size = System::Drawing::Size(56, 20);
			this->nbxTheta6->TabIndex = 5;
			this->nbxTheta6->Text = L"90.0";
			this->nbxTheta6->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// trkTheta6
			// 
			this->trkTheta6->Location = System::Drawing::Point(6, 19);
			this->trkTheta6->Maximum = 1800;
			this->trkTheta6->Name = L"trkTheta6";
			this->trkTheta6->Size = System::Drawing::Size(126, 45);
			this->trkTheta6->TabIndex = 1;
			this->trkTheta6->Value = 900;
			this->trkTheta6->Scroll += gcnew System::EventHandler(this, &MyForm::trkTheta6_Scroll);
			// 
			// groupBox11
			// 
			this->groupBox11->Controls->Add(this->nbxTheta3);
			this->groupBox11->Controls->Add(this->trkTheta3);
			this->groupBox11->Location = System::Drawing::Point(6, 159);
			this->groupBox11->Name = L"groupBox11";
			this->groupBox11->Size = System::Drawing::Size(200, 64);
			this->groupBox11->TabIndex = 0;
			this->groupBox11->TabStop = false;
			this->groupBox11->Text = L"Theta 3";
			// 
			// nbxTheta3
			// 
			this->nbxTheta3->Cursor = System::Windows::Forms::Cursors::Default;
			this->nbxTheta3->Location = System::Drawing::Point(138, 19);
			this->nbxTheta3->Name = L"nbxTheta3";
			this->nbxTheta3->ReadOnly = true;
			this->nbxTheta3->Size = System::Drawing::Size(56, 20);
			this->nbxTheta3->TabIndex = 5;
			this->nbxTheta3->Text = L"90.0";
			this->nbxTheta3->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// trkTheta3
			// 
			this->trkTheta3->Location = System::Drawing::Point(6, 19);
			this->trkTheta3->Maximum = 1800;
			this->trkTheta3->Name = L"trkTheta3";
			this->trkTheta3->Size = System::Drawing::Size(126, 45);
			this->trkTheta3->TabIndex = 1;
			this->trkTheta3->Value = 900;
			this->trkTheta3->Scroll += gcnew System::EventHandler(this, &MyForm::trkTheta3_Scroll);
			// 
			// groupBox9
			// 
			this->groupBox9->Controls->Add(this->nbxTheta2);
			this->groupBox9->Controls->Add(this->trkTheta2);
			this->groupBox9->Location = System::Drawing::Point(6, 89);
			this->groupBox9->Name = L"groupBox9";
			this->groupBox9->Size = System::Drawing::Size(200, 64);
			this->groupBox9->TabIndex = 0;
			this->groupBox9->TabStop = false;
			this->groupBox9->Text = L"Theta 2";
			// 
			// nbxTheta2
			// 
			this->nbxTheta2->Cursor = System::Windows::Forms::Cursors::Default;
			this->nbxTheta2->Location = System::Drawing::Point(138, 19);
			this->nbxTheta2->Name = L"nbxTheta2";
			this->nbxTheta2->ReadOnly = true;
			this->nbxTheta2->Size = System::Drawing::Size(56, 20);
			this->nbxTheta2->TabIndex = 5;
			this->nbxTheta2->Text = L"90.0";
			this->nbxTheta2->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// trkTheta2
			// 
			this->trkTheta2->Location = System::Drawing::Point(6, 19);
			this->trkTheta2->Maximum = 1800;
			this->trkTheta2->Name = L"trkTheta2";
			this->trkTheta2->Size = System::Drawing::Size(126, 45);
			this->trkTheta2->TabIndex = 1;
			this->trkTheta2->Value = 900;
			this->trkTheta2->Scroll += gcnew System::EventHandler(this, &MyForm::trkTheta2_Scroll);
			// 
			// groupBox7
			// 
			this->groupBox7->Controls->Add(this->nbxTheta1);
			this->groupBox7->Controls->Add(this->trkTheta1);
			this->groupBox7->Location = System::Drawing::Point(6, 19);
			this->groupBox7->Name = L"groupBox7";
			this->groupBox7->Size = System::Drawing::Size(200, 64);
			this->groupBox7->TabIndex = 0;
			this->groupBox7->TabStop = false;
			this->groupBox7->Text = L"Theta 1";
			// 
			// nbxTheta1
			// 
			this->nbxTheta1->Cursor = System::Windows::Forms::Cursors::Default;
			this->nbxTheta1->Location = System::Drawing::Point(138, 19);
			this->nbxTheta1->Name = L"nbxTheta1";
			this->nbxTheta1->ReadOnly = true;
			this->nbxTheta1->Size = System::Drawing::Size(56, 20);
			this->nbxTheta1->TabIndex = 5;
			this->nbxTheta1->Text = L"90.0";
			this->nbxTheta1->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// trkTheta1
			// 
			this->trkTheta1->Location = System::Drawing::Point(6, 19);
			this->trkTheta1->Maximum = 1800;
			this->trkTheta1->Name = L"trkTheta1";
			this->trkTheta1->Size = System::Drawing::Size(126, 45);
			this->trkTheta1->TabIndex = 1;
			this->trkTheta1->Value = 900;
			this->trkTheta1->Scroll += gcnew System::EventHandler(this, &MyForm::trkTheta1_Scroll);
			// 
			// grpInv
			// 
			this->grpInv->Controls->Add(this->groupBox14);
			this->grpInv->Controls->Add(this->groupBox15);
			this->grpInv->Controls->Add(this->groupBox16);
			this->grpInv->Controls->Add(this->groupBox17);
			this->grpInv->Controls->Add(this->groupBox18);
			this->grpInv->Controls->Add(this->groupBox19);
			this->grpInv->Enabled = false;
			this->grpInv->Location = System::Drawing::Point(274, 402);
			this->grpInv->Name = L"grpInv";
			this->grpInv->Size = System::Drawing::Size(423, 228);
			this->grpInv->TabIndex = 4;
			this->grpInv->TabStop = false;
			this->grpInv->Text = L"Inverse Kinematic";
			// 
			// groupBox14
			// 
			this->groupBox14->Controls->Add(this->nbxPosRoll);
			this->groupBox14->Controls->Add(this->trkPosRoll);
			this->groupBox14->Location = System::Drawing::Point(217, 19);
			this->groupBox14->Name = L"groupBox14";
			this->groupBox14->Size = System::Drawing::Size(200, 64);
			this->groupBox14->TabIndex = 0;
			this->groupBox14->TabStop = false;
			this->groupBox14->Text = L"Roll";
			// 
			// nbxPosRoll
			// 
			this->nbxPosRoll->Cursor = System::Windows::Forms::Cursors::Default;
			this->nbxPosRoll->Location = System::Drawing::Point(138, 19);
			this->nbxPosRoll->Name = L"nbxPosRoll";
			this->nbxPosRoll->ReadOnly = true;
			this->nbxPosRoll->Size = System::Drawing::Size(56, 20);
			this->nbxPosRoll->TabIndex = 5;
			this->nbxPosRoll->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// trkPosRoll
			// 
			this->trkPosRoll->Location = System::Drawing::Point(6, 19);
			this->trkPosRoll->Maximum = 1800;
			this->trkPosRoll->Name = L"trkPosRoll";
			this->trkPosRoll->Size = System::Drawing::Size(126, 45);
			this->trkPosRoll->TabIndex = 1;
			this->trkPosRoll->Value = 900;
			this->trkPosRoll->Scroll += gcnew System::EventHandler(this, &MyForm::trkPosRoll_Scroll);
			// 
			// groupBox15
			// 
			this->groupBox15->Controls->Add(this->nbxPosPitch);
			this->groupBox15->Controls->Add(this->trkPosPitch);
			this->groupBox15->Location = System::Drawing::Point(217, 89);
			this->groupBox15->Name = L"groupBox15";
			this->groupBox15->Size = System::Drawing::Size(200, 64);
			this->groupBox15->TabIndex = 0;
			this->groupBox15->TabStop = false;
			this->groupBox15->Text = L"Pitch";
			// 
			// nbxPosPitch
			// 
			this->nbxPosPitch->Cursor = System::Windows::Forms::Cursors::Default;
			this->nbxPosPitch->Location = System::Drawing::Point(138, 19);
			this->nbxPosPitch->Name = L"nbxPosPitch";
			this->nbxPosPitch->ReadOnly = true;
			this->nbxPosPitch->Size = System::Drawing::Size(56, 20);
			this->nbxPosPitch->TabIndex = 5;
			this->nbxPosPitch->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// trkPosPitch
			// 
			this->trkPosPitch->Location = System::Drawing::Point(6, 19);
			this->trkPosPitch->Maximum = 1800;
			this->trkPosPitch->Name = L"trkPosPitch";
			this->trkPosPitch->Size = System::Drawing::Size(126, 45);
			this->trkPosPitch->TabIndex = 1;
			this->trkPosPitch->Value = 900;
			this->trkPosPitch->Scroll += gcnew System::EventHandler(this, &MyForm::trkPosPitch_Scroll);
			// 
			// groupBox16
			// 
			this->groupBox16->Controls->Add(this->nbxPosGrip);
			this->groupBox16->Controls->Add(this->trkPosGrip);
			this->groupBox16->Location = System::Drawing::Point(217, 159);
			this->groupBox16->Name = L"groupBox16";
			this->groupBox16->Size = System::Drawing::Size(200, 64);
			this->groupBox16->TabIndex = 0;
			this->groupBox16->TabStop = false;
			this->groupBox16->Text = L"Gripper";
			// 
			// nbxPosGrip
			// 
			this->nbxPosGrip->Cursor = System::Windows::Forms::Cursors::Default;
			this->nbxPosGrip->Location = System::Drawing::Point(138, 19);
			this->nbxPosGrip->Name = L"nbxPosGrip";
			this->nbxPosGrip->ReadOnly = true;
			this->nbxPosGrip->Size = System::Drawing::Size(56, 20);
			this->nbxPosGrip->TabIndex = 5;
			this->nbxPosGrip->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// trkPosGrip
			// 
			this->trkPosGrip->Location = System::Drawing::Point(6, 19);
			this->trkPosGrip->Maximum = 1800;
			this->trkPosGrip->Name = L"trkPosGrip";
			this->trkPosGrip->Size = System::Drawing::Size(126, 45);
			this->trkPosGrip->TabIndex = 1;
			this->trkPosGrip->Value = 900;
			this->trkPosGrip->Scroll += gcnew System::EventHandler(this, &MyForm::trkPosGrip_Scroll);
			// 
			// groupBox17
			// 
			this->groupBox17->Controls->Add(this->nbxPosZ);
			this->groupBox17->Controls->Add(this->trkPosZ);
			this->groupBox17->Location = System::Drawing::Point(6, 159);
			this->groupBox17->Name = L"groupBox17";
			this->groupBox17->Size = System::Drawing::Size(200, 64);
			this->groupBox17->TabIndex = 0;
			this->groupBox17->TabStop = false;
			this->groupBox17->Text = L"Position Z";
			// 
			// nbxPosZ
			// 
			this->nbxPosZ->Cursor = System::Windows::Forms::Cursors::Default;
			this->nbxPosZ->Location = System::Drawing::Point(138, 19);
			this->nbxPosZ->Name = L"nbxPosZ";
			this->nbxPosZ->ReadOnly = true;
			this->nbxPosZ->Size = System::Drawing::Size(56, 20);
			this->nbxPosZ->TabIndex = 5;
			this->nbxPosZ->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// trkPosZ
			// 
			this->trkPosZ->Location = System::Drawing::Point(6, 19);
			this->trkPosZ->Maximum = 3500;
			this->trkPosZ->Minimum = -3500;
			this->trkPosZ->Name = L"trkPosZ";
			this->trkPosZ->Size = System::Drawing::Size(126, 45);
			this->trkPosZ->TabIndex = 1;
			this->trkPosZ->Scroll += gcnew System::EventHandler(this, &MyForm::trkPosZ_Scroll);
			// 
			// groupBox18
			// 
			this->groupBox18->Controls->Add(this->nbxPosY);
			this->groupBox18->Controls->Add(this->trkPosY);
			this->groupBox18->Location = System::Drawing::Point(6, 89);
			this->groupBox18->Name = L"groupBox18";
			this->groupBox18->Size = System::Drawing::Size(200, 64);
			this->groupBox18->TabIndex = 0;
			this->groupBox18->TabStop = false;
			this->groupBox18->Text = L"Posistion Y";
			// 
			// nbxPosY
			// 
			this->nbxPosY->Cursor = System::Windows::Forms::Cursors::Default;
			this->nbxPosY->Location = System::Drawing::Point(138, 19);
			this->nbxPosY->Name = L"nbxPosY";
			this->nbxPosY->ReadOnly = true;
			this->nbxPosY->Size = System::Drawing::Size(56, 20);
			this->nbxPosY->TabIndex = 5;
			this->nbxPosY->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// trkPosY
			// 
			this->trkPosY->Location = System::Drawing::Point(6, 19);
			this->trkPosY->Maximum = 3500;
			this->trkPosY->Minimum = -3500;
			this->trkPosY->Name = L"trkPosY";
			this->trkPosY->Size = System::Drawing::Size(126, 45);
			this->trkPosY->TabIndex = 1;
			this->trkPosY->Scroll += gcnew System::EventHandler(this, &MyForm::trkPosY_Scroll);
			// 
			// groupBox19
			// 
			this->groupBox19->Controls->Add(this->nbxPosX);
			this->groupBox19->Controls->Add(this->trkPosX);
			this->groupBox19->Location = System::Drawing::Point(6, 19);
			this->groupBox19->Name = L"groupBox19";
			this->groupBox19->Size = System::Drawing::Size(200, 64);
			this->groupBox19->TabIndex = 0;
			this->groupBox19->TabStop = false;
			this->groupBox19->Text = L"Posistion X";
			// 
			// nbxPosX
			// 
			this->nbxPosX->Cursor = System::Windows::Forms::Cursors::Default;
			this->nbxPosX->Location = System::Drawing::Point(138, 19);
			this->nbxPosX->Name = L"nbxPosX";
			this->nbxPosX->ReadOnly = true;
			this->nbxPosX->Size = System::Drawing::Size(56, 20);
			this->nbxPosX->TabIndex = 5;
			this->nbxPosX->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// trkPosX
			// 
			this->trkPosX->Location = System::Drawing::Point(6, 19);
			this->trkPosX->Maximum = 3500;
			this->trkPosX->Minimum = -3500;
			this->trkPosX->Name = L"trkPosX";
			this->trkPosX->Size = System::Drawing::Size(126, 45);
			this->trkPosX->TabIndex = 1;
			this->trkPosX->Scroll += gcnew System::EventHandler(this, &MyForm::trkPosX_Scroll);
			// 
			// timer1
			// 
			this->timer1->Enabled = true;
			this->timer1->Interval = 50;
			this->timer1->Tick += gcnew System::EventHandler(this, &MyForm::timer1_Tick);
			// 
			// groupBox6
			// 
			this->groupBox6->Controls->Add(this->richTextBox1);
			this->groupBox6->Location = System::Drawing::Point(280, 12);
			this->groupBox6->Name = L"groupBox6";
			this->groupBox6->Size = System::Drawing::Size(417, 100);
			this->groupBox6->TabIndex = 5;
			this->groupBox6->TabStop = false;
			this->groupBox6->Text = L"Project Infomation";
			// 
			// richTextBox1
			// 
			this->richTextBox1->Cursor = System::Windows::Forms::Cursors::Default;
			this->richTextBox1->ImeMode = System::Windows::Forms::ImeMode::NoControl;
			this->richTextBox1->Location = System::Drawing::Point(6, 19);
			this->richTextBox1->Name = L"richTextBox1";
			this->richTextBox1->ReadOnly = true;
			this->richTextBox1->Size = System::Drawing::Size(405, 75);
			this->richTextBox1->TabIndex = 0;
			this->richTextBox1->Text = L"Project Name\t: Robotic Arm Model for Palletizing Objects\nStudent\t\t: Tạ Nguyên Bảo"
				L"\nSupervisor\t: Nguyễn Đức Hoàng\nFaculty\t\t: Faculty of Electrical and Electronics "
				L"Engineering, HCMUT";
			// 
			// groupBox13
			// 
			this->groupBox13->Controls->Add(this->radioButton3);
			this->groupBox13->Controls->Add(this->radioButton2);
			this->groupBox13->Controls->Add(this->radioButton1);
			this->groupBox13->Location = System::Drawing::Point(280, 112);
			this->groupBox13->Name = L"groupBox13";
			this->groupBox13->Size = System::Drawing::Size(417, 45);
			this->groupBox13->TabIndex = 6;
			this->groupBox13->TabStop = false;
			this->groupBox13->Text = L"Robot Process";
			// 
			// radioButton3
			// 
			this->radioButton3->AutoSize = true;
			this->radioButton3->Location = System::Drawing::Point(286, 22);
			this->radioButton3->Name = L"radioButton3";
			this->radioButton3->Size = System::Drawing::Size(71, 17);
			this->radioButton3->TabIndex = 0;
			this->radioButton3->TabStop = true;
			this->radioButton3->Text = L"Robot On";
			this->radioButton3->UseVisualStyleBackColor = true;
			// 
			// radioButton2
			// 
			this->radioButton2->AutoSize = true;
			this->radioButton2->Location = System::Drawing::Point(166, 19);
			this->radioButton2->Name = L"radioButton2";
			this->radioButton2->Size = System::Drawing::Size(71, 17);
			this->radioButton2->TabIndex = 0;
			this->radioButton2->TabStop = true;
			this->radioButton2->Text = L"Robot On";
			this->radioButton2->UseVisualStyleBackColor = true;
			// 
			// radioButton1
			// 
			this->radioButton1->AutoSize = true;
			this->radioButton1->Location = System::Drawing::Point(52, 22);
			this->radioButton1->Name = L"radioButton1";
			this->radioButton1->Size = System::Drawing::Size(71, 17);
			this->radioButton1->TabIndex = 0;
			this->radioButton1->TabStop = true;
			this->radioButton1->Text = L"Robot On";
			this->radioButton1->UseVisualStyleBackColor = true;
			// 
			// MyForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(705, 636);
			this->Controls->Add(this->groupBox13);
			this->Controls->Add(this->groupBox6);
			this->Controls->Add(this->grpInv);
			this->Controls->Add(this->grpFwd);
			this->Controls->Add(this->grpControlPanel);
			this->Controls->Add(this->groupBox2);
			this->Controls->Add(this->groupBox1);
			this->Icon = (cli::safe_cast<System::Drawing::Icon^>(resources->GetObject(L"$this.Icon")));
			this->Name = L"MyForm";
			this->Text = L"Robot Arm Control";
			this->Load += gcnew System::EventHandler(this, &MyForm::MyForm_Load);
			this->groupBox1->ResumeLayout(false);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			this->groupBox2->ResumeLayout(false);
			this->grpControlPanel->ResumeLayout(false);
			this->groupBox5->ResumeLayout(false);
			this->groupBox4->ResumeLayout(false);
			this->groupBox3->ResumeLayout(false);
			this->grpFwd->ResumeLayout(false);
			this->groupBox8->ResumeLayout(false);
			this->groupBox8->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkTheta4))->EndInit();
			this->groupBox10->ResumeLayout(false);
			this->groupBox10->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkTheta5))->EndInit();
			this->groupBox12->ResumeLayout(false);
			this->groupBox12->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkTheta6))->EndInit();
			this->groupBox11->ResumeLayout(false);
			this->groupBox11->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkTheta3))->EndInit();
			this->groupBox9->ResumeLayout(false);
			this->groupBox9->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkTheta2))->EndInit();
			this->groupBox7->ResumeLayout(false);
			this->groupBox7->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkTheta1))->EndInit();
			this->grpInv->ResumeLayout(false);
			this->groupBox14->ResumeLayout(false);
			this->groupBox14->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkPosRoll))->EndInit();
			this->groupBox15->ResumeLayout(false);
			this->groupBox15->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkPosPitch))->EndInit();
			this->groupBox16->ResumeLayout(false);
			this->groupBox16->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkPosGrip))->EndInit();
			this->groupBox17->ResumeLayout(false);
			this->groupBox17->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkPosZ))->EndInit();
			this->groupBox18->ResumeLayout(false);
			this->groupBox18->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkPosY))->EndInit();
			this->groupBox19->ResumeLayout(false);
			this->groupBox19->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trkPosX))->EndInit();
			this->groupBox6->ResumeLayout(false);
			this->groupBox13->ResumeLayout(false);
			this->groupBox13->PerformLayout();
			this->ResumeLayout(false);

		}
#pragma endregion
	private: TcpClient^ client;
	private: bool isCameraRunning;
	private: bool isRobotRunning;
	private: bool isForwardMode;
	private: bool isChanged;
	private: double dTheta1, dTheta2, dTheta3, dTheta4, dTheta5, dTheta6;
	private: int iPosX, iPosY, iPosZ, iPosRoll, iPosPitch, iPosGrip;
	private: int iTheta1, iTheta2, iTheta3, iTheta4, iTheta5, iTheta6;
	private: int iCenterX = 0;
		   int iCenterY = 0;
		   int iAngle;
	private: double dPosX, dPosY, dPosZ, dPosRoll, dPosPitch, dPosGrip;
	private: NetworkStream^ stream;
		   Thread^ cameraThread;
		   HttpWebRequest^ httpRequest;
		   HttpWebResponse^ httpResponse;

	private: System::Drawing::Image^ esp32Image;


	private: System::Void MyForm_Load(System::Object^ sender, System::EventArgs^ e) {
		try {
			timer1->Stop();
			cbxMode->SelectedIndex = 0;
			// Execute the arp -a command and capture the output
			std::array<char, 128> buffer;
			std::string result;
			std::shared_ptr<FILE> pipe(_popen("arp -a", "r"), _pclose);
			if (!pipe) throw std::runtime_error("popen() failed!");
			while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
				result += buffer.data();
			}

			// Split the result into lines and add them to the ComboBox
			std::istringstream iss(result);
			std::string line;
			std::string delimiter = " ";
			while (std::getline(iss, line)) {
				size_t pos = 0;
				while ((pos = line.find(delimiter)) != std::string::npos) {
					std::string token = line.substr(0, pos);
					line.erase(0, pos + delimiter.length());
					if (std::count(token.begin(), token.end(), '.') == 3) {
						std::string prefix = token.substr(0, token.rfind('.'));
						int lastOctet = std::stoi(token.substr(token.rfind('.') + 1));
						if ((prefix == "192.168.137" || prefix == "192.168.1" || prefix == "192.168.0") && (lastOctet >= 2 && lastOctet <= 254)) {
							System::String^ output = gcnew System::String(token.c_str());
							cbxConnection->Items->Add(output);
						}
					}
				}
			}

			if (cbxConnection->Items->Count > 0) {
				cbxConnection->SelectedIndex = 0;
			}
			else {
				MessageBox::Show("No IP address", "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
			}
			ThetaScroll();
		}
		catch (const std::exception& ex) {
			MessageBox::Show(gcnew System::String(ex.what()), "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
		}
	}
    private: System::Void btn_Connect_Click(System::Object^ sender, System::EventArgs^ e) {
        try {
            timer1->Enabled = true;
            System::String^ selectedIP;
            if (cbxConnection->SelectedIndex == -2) {

                MessageBox::Show("Please select an IP address", "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
                return;
            }
            else if (cbxConnection->Text->Equals("esp32cam.local")) {
                System::Net::IPHostEntry^ hostEntry = System::Net::Dns::GetHostEntry("esp32cam.local");
                selectedIP = hostEntry->AddressList[0]->ToString();
            }
            else {
                selectedIP = cbxConnection->Text;
            }
            // Enable the control panels
            grpControlPanel->Enabled = true;
            btn_Connect->ForeColor = System::Drawing::Color::Red;
            btn_Connect->Text = "Disconnect";
        }
        catch (System::Exception^ ex) {
            MessageBox::Show("Failed to connect: " + ex->Message, "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
        }
    }
	private: System::Void btnModeProcess_Click(System::Object^ sender, System::EventArgs^ e) {
		if (cbxMode->SelectedIndex == 0) {
			grpFwd->Enabled = true;
			grpInv->Enabled = true;
			timer1->Enabled = true;
			groupBox5->Enabled = false;
			groupBox4->Enabled = true;
			timer1->Start();
		}
		else {
			grpFwd->Enabled = false;
			grpInv->Enabled = false;
			groupBox5->Enabled = true;
			groupBox4->Enabled = false;
			timer1->Enabled = true;
			timer1->Stop();
		}
	}

	private:
        void ForwardKinematic() {  
           isForwardMode = true;  
           double theta1 = dTheta1 * M_PI / 180;  
           double theta2 = dTheta2 * M_PI / 180;  
           double theta3 = dTheta3 * M_PI / 180;  
           double theta4 = dTheta4 * M_PI / 180;  
           double theta5 = dTheta5 * M_PI / 180;  
           double theta6 = dTheta6 * M_PI / 180;  

           dPosX = 25 * cos(theta1) * (6 * cos(theta2 + theta3 + theta4) - sin(theta2 + theta3 + theta4) + 4 * cos(theta2 + theta3) + 4 * cos(theta2));  
           dPosY = 25 * sin(theta1) * (6 * cos(theta2 + theta3 + theta4) - sin(theta2 + theta3 + theta4) + 4 * cos(theta2 + theta3) + 4 * cos(theta2));  
           dPosZ = 100 * sin(theta2 + theta3) + 100 * sin(theta2) + 25 * sqrt(37) * cos(theta2 + theta3 + theta4 - atan(6));  
           dPosPitch = dTheta2 + dTheta3 + dTheta4;  
           dPosRoll = dTheta5;  
           dPosGrip = dTheta6;  

           // Round the calculated positions to 2 decimal places  
           dPosX = round(dPosX * 100) / 100.0;  
           dPosY = round(dPosY * 100) / 100.0;  
           dPosZ = round(dPosZ * 100) / 100.0;  
           dPosPitch = round(dPosPitch * 100) / 100.0;  
           dPosRoll = round(dPosRoll * 100) / 100.0;  
           dPosGrip = round(dPosGrip * 100) / 100.0;  

           while (dPosPitch > 180) {  
               dPosPitch -= 180;  
           }  
        }

		void InverseKinematic() {
			isForwardMode = true;

			double x = dPosX;
			double y = dPosY;
			double z = dPosZ;

			double r = sqrt(x * x + y * y);
			double pwr = r - 25;
			double pwz = z + 150;
			double a = sqrt(pwr * pwr + pwz * pwz);
			a = round(a * 10) / 10;

			if (a > 200) {
				MessageBox::Show("Position out of reach", "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
				return;
			}
			else if (a == 200) {
				MessageBox::Show("Position at the edge of reach", "Warning", MessageBoxButtons::OK, MessageBoxIcon::Warning);
				dTheta3 = 0;
			}
			else if (a == 0) {
				MessageBox::Show("Position at the base", "Warning", MessageBoxButtons::OK, MessageBoxIcon::Warning);
				dTheta3 = M_PI;
			}

			dTheta3 = acos((a * a - 100 * 100 - 100 * 100) / (-2 * 100 * 100));
			double alpha = (M_PI - dTheta3) / 2;
			dTheta3 = M_PI - dTheta3;
			dTheta2 = M_PI - (atan2(pwz, pwr) + acos((100 * 100 + a * a - 100 * 100) / (2 * 100 * a)));
			dTheta4 = M_PI - (atan2(pwr, pwz) + alpha);
			dTheta1 = atan2(y, x);
			dTheta5 = dPosRoll;
			dTheta6 = dPosGrip;
			dTheta1 = round(dTheta1 * 180 / M_PI * 10) / 10;
			dTheta2 = round(dTheta2 * 180 / M_PI * 10) / 10;
			dTheta3 = round(dTheta3 * 180 / M_PI * 10) / 10;
			dTheta4 = round(dTheta4 * 180 / M_PI * 10) / 10;
			if (dTheta1 < 0) {
				dTheta1 += M_PI;
			}
		}
		void ThetaScroll()
		{
			timer1->Enabled = true;
			isChanged = true;
			iTheta1 = trkTheta1->Value;
			iTheta2 = trkTheta2->Value;
			iTheta3 = trkTheta3->Value;
			//iTheta4 = trkTheta4->Value;
			iTheta4 = iTheta2 + 840 - iTheta3;
			trkTheta4->Value = iTheta4;
			iTheta5 = trkTheta5->Value;
			iTheta6 = trkTheta6->Value;
			dTheta1 = iTheta1 / 10.0;
			dTheta2 = iTheta2 / 10.0;
			dTheta3 = iTheta3 / 10.0;
			dTheta4 = iTheta4 / 10.0;
			dTheta5 = iTheta5 / 10.0;
			dTheta6 = iTheta6 / 10.0;
			ForwardKinematic();
			nbxTheta1->Text = dTheta1.ToString();
			nbxTheta2->Text = dTheta2.ToString();
			nbxTheta3->Text = dTheta3.ToString();
			nbxTheta4->Text = dTheta4.ToString();
			nbxTheta5->Text = dTheta5.ToString();
			nbxTheta6->Text = dTheta6.ToString();
			nbxPosX->Text = dPosX.ToString();
			nbxPosY->Text = dPosY.ToString();
			nbxPosZ->Text = dPosZ.ToString();
			nbxPosRoll->Text = dPosRoll.ToString();
			nbxPosPitch->Text = dPosPitch.ToString();
			nbxPosGrip->Text = dPosGrip.ToString();
			iPosX = (int)(dPosX * 10);
			iPosY = (int)(dPosY * 10);
			iPosZ = (int)(dPosZ * 10);
			iPosRoll = (int)(dPosRoll * 10);
			iPosPitch = (int)(dPosPitch * 10);
			iPosGrip = (int)(dPosGrip * 10);
			trkPosX->Value = iPosX;
			trkPosY->Value = iPosY;
			trkPosZ->Value = iPosZ;
			trkPosRoll->Value = iPosRoll;
			trkPosPitch->Value = iPosPitch;
			trkPosGrip->Value = iPosGrip;
		}
		void PositionScroll()
		{
			timer1->Enabled = true; 
			isChanged = true;
			iPosX = trkPosX->Value;
			iPosY = trkPosY->Value;
			iPosZ = trkPosZ->Value;
			iPosRoll = trkPosRoll->Value;
			iPosPitch = trkPosPitch->Value;
			iPosGrip = trkPosGrip->Value;
			dPosX = iPosX / 10.0;
			dPosY = iPosY / 10.0;
			dPosZ = iPosZ / 10.0;
			dPosRoll = iPosRoll / 10.0;
			dPosPitch = iPosPitch / 10.0;
			dPosGrip = iPosGrip / 10.0;
			InverseKinematic();
			nbxPosX->Text = dPosX.ToString();
			nbxPosY->Text = dPosY.ToString();
			nbxPosZ->Text = dPosZ.ToString();
			nbxPosRoll->Text = dPosRoll.ToString();
			nbxPosPitch->Text = dPosPitch.ToString();
			nbxPosGrip->Text = dPosGrip.ToString();
			iTheta1 = (int)(dTheta1 * 10);
			iTheta2 = (int)(dTheta2 * 10);
			iTheta3 = (int)(dTheta3 * 10);
			iTheta4 = (int)(dTheta4 * 10);
			iTheta5 = (int)(dTheta5 * 10);
			iTheta6 = (int)(dTheta6 * 10);
			try
			{
				trkTheta1->Value = iTheta1;
				trkTheta2->Value = iTheta2;
				trkTheta3->Value = iTheta3;
				trkTheta4->Value = iTheta4;
				trkTheta5->Value = iTheta5;
				trkTheta6->Value = iTheta6;
			}
			catch (System::Exception^)
			{
				MessageBox::Show("Out of Reach", "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
			}
			nbxTheta1->Text = dTheta1.ToString();
			nbxTheta2->Text = dTheta2.ToString();
			nbxTheta3->Text = dTheta3.ToString();
			nbxTheta4->Text = dTheta4.ToString();
			nbxTheta5->Text = dTheta5.ToString();
			nbxTheta6->Text = dTheta6.ToString();
		}
	private: System::Void trkTheta1_Scroll(System::Object^ sender, System::EventArgs^ e) {
		ThetaScroll();
	}
	private: System::Void trkTheta2_Scroll(System::Object^ sender, System::EventArgs^ e) {
		ThetaScroll();
	}
	private: System::Void trkTheta4_Scroll(System::Object^ sender, System::EventArgs^ e) {
		ThetaScroll();
	}
	private: System::Void trkTheta5_Scroll(System::Object^ sender, System::EventArgs^ e) {
		ThetaScroll();
	}
	private: System::Void trkTheta3_Scroll(System::Object^ sender, System::EventArgs^ e) {
		ThetaScroll();
	}
	private: System::Void trkTheta6_Scroll(System::Object^ sender, System::EventArgs^ e) {
		ThetaScroll();
	}
	private: System::Void trkPosX_Scroll(System::Object^ sender, System::EventArgs^ e) {
		PositionScroll();
	}
	private: System::Void trkPosY_Scroll(System::Object^ sender, System::EventArgs^ e) {
		PositionScroll();
	}
	private: System::Void trkPosZ_Scroll(System::Object^ sender, System::EventArgs^ e) {
		PositionScroll();
	}
	private: System::Void trkPosRoll_Scroll(System::Object^ sender, System::EventArgs^ e) {
		PositionScroll();
	}
	private: System::Void trkPosPitch_Scroll(System::Object^ sender, System::EventArgs^ e) {
		PositionScroll();
	}
	private: System::Void trkPosGrip_Scroll(System::Object^ sender, System::EventArgs^ e) {
		PositionScroll();
	}
	private: System::Void btnFlash_Click(System::Object^ sender, System::EventArgs^ e) {
		if (client != nullptr && client->Connected) {
			try {
				System::String^ url;
				if (btnFlash->Text == "Flash On") {
					url = "http://" + cbxConnection->Text + "/on";
				}
				else {
					url = "http://" + cbxConnection->Text + "/off";
				}

				httpRequest = dynamic_cast<HttpWebRequest^>(WebRequest::Create(url));
				httpResponse = dynamic_cast<HttpWebResponse^>(httpRequest->GetResponse());

				// Dispose of the response object to free up resources
				httpResponse->Close();

				// Toggle the button text
				btnFlash->Text = (btnFlash->Text == "Flash On") ? "Flash Off" : "Flash On";
			}
			catch (System::Exception^ ex) {
				MessageBox::Show("Error Flash Led: " + ex->Message);
			}
		}
	}
	private: System::Void timer1_Tick(System::Object^ sender, System::EventArgs^ e) {
		if (client != nullptr && client->Connected) {
			FetchAndUpdateImageAsync(cbxConnection->Text, pictureBox1);
		}
	}

	private: Task^ FetchAndUpdateImageAsync(System::String^ connectionText, PictureBox^ pictureBox) {
		return Task::Run(gcnew Func<Task^>(this, &MyForm::FetchAndUpdateImage));
	}

    private: void DetectAndDrawWhiteRectangle(cv::Mat& frame) {
        // Resize the image
        cv::Mat image_resized;
        cv::resize(frame, image_resized, cv::Size(240, 240));

        // Convert the image to grayscale
        cv::Mat gray;
        cv::cvtColor(image_resized, gray, cv::COLOR_BGR2GRAY);

        // Apply GaussianBlur to reduce noise
        cv::Mat blurred;
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

        // Apply threshold to create a binary image
        cv::Mat thresh;
        cv::threshold(blurred, thresh, 160, 255, cv::THRESH_BINARY);

        // Use morphological opening to remove small white regions
        cv::Mat opening;
        cv::morphologyEx(thresh, opening, cv::MORPH_OPEN, cv::Mat::ones(cv::Size(5, 5), CV_8U));

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(opening, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Variables to store the largest rectangle
        cv::RotatedRect largest_rect;
        double largest_area = 0;

        // Iterate through the contours to find the largest rectangle
        for (const auto& contour : contours) {
            // Ignore small contours
            if (cv::contourArea(contour) < 1000) {
                continue;
            }

            // Calculate the bounding rectangle
            cv::RotatedRect rect = cv::minAreaRect(contour);
            double area = rect.size.width * rect.size.height;

            // Check if the area is larger than the current largest area
            if (area > largest_area) {
                largest_area = area;
                largest_rect = rect;
            }
        }

        // Draw the largest rectangle if found
        if (largest_area > 0) {
            cv::Point2f box_points[4];
            largest_rect.points(box_points);
            for (int i = 0; i < 4; ++i) {
                cv::line(image_resized, box_points[i], box_points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
            }

            // Calculate the center, width, height, and angle of the rectangle
            cv::Point center = largest_rect.center;
            int width = static_cast<int>(largest_rect.size.width);
            int height = static_cast<int>(largest_rect.size.height);
            double angle = largest_rect.angle;

            // Update the class variables
            iCenterX = center.x;
            iCenterY = center.y;
            iAngle = static_cast<int>(angle * 10);

            // Draw the center of the rectangle
            cv::circle(image_resized, center, 5, cv::Scalar(0, 0, 255), -1);
        }

        // Convert the processed image to a format suitable for PictureBox
        System::Drawing::Graphics^ graphics = pictureBox1->CreateGraphics();
        System::IntPtr ptr(image_resized.ptr());
        System::Drawing::Bitmap^ b = gcnew System::Drawing::Bitmap(image_resized.cols, image_resized.rows, image_resized.step, System::Drawing::Imaging::PixelFormat::Format24bppRgb, ptr);
        System::Drawing::RectangleF rect(0, 0, pictureBox1->Width, pictureBox1->Height);
        graphics->DrawImage(b, rect);
    }

    private: Task^ FetchAndUpdateImage() {
        try {
            int cmd, value[6];
            if (isRobotRunning)
            {
                cmd = _RUNROBOT_;
                value[0] = iCenterX;
                value[1] = iCenterY;
                value[2] = iAngle;
                value[3] = 0;
                value[4] = 0;
                value[5] = 0;
            }
            else if (isForwardMode)
            {
                cmd = _SET_ANGLE_;
                value[0] = iTheta1;
                value[1] = iTheta2;
                value[2] = iTheta3;
                value[3] = iTheta4;
                value[4] = iTheta5;
                value[5] = iTheta6;
            }
            else
            {
                cmd = _SET_ANGLE_;
                value[0] = iTheta1;
                value[1] = iTheta2;
                value[2] = iTheta3;
                value[3] = iTheta4;
                value[4] = iTheta5;
                value[5] = iTheta6;
            }
            System::String^ url = "http://" + this->Invoke(gcnew Func<System::String^>(this, &MyForm::GetConnectionText)) + "/robot?cmd=" + cmd.ToString() +
                "&value1=" + value[0].ToString() + "&value2=" + value[1].ToString() + "&value3=" + value[2].ToString() +
                "&value4=" + value[3].ToString() + "&value5=" + value[4].ToString() + "&value6=" + value[5].ToString();
            HttpWebRequest^ request;
            HttpWebResponse^ response;
            System::Drawing::Image^ img;
            if (isRobotRunning)
            {
                request = dynamic_cast<HttpWebRequest^>(WebRequest::Create(url));
                response = dynamic_cast<HttpWebResponse^>(request->GetResponse());

                img = System::Drawing::Image::FromStream(response->GetResponseStream());
                // Dispose of the response object to free up resources
                response->Close();

                // Convert System::Drawing::Image to OpenCV Mat
                System::Drawing::Bitmap^ bmp = dynamic_cast<System::Drawing::Bitmap^>(img);
                System::Drawing::Imaging::BitmapData^ bmpData = bmp->LockBits(System::Drawing::Rectangle(0, 0, bmp->Width, bmp->Height), System::Drawing::Imaging::ImageLockMode::ReadOnly, bmp->PixelFormat);
                cv::Mat mat(bmp->Height, bmp->Width, CV_8UC3, bmpData->Scan0.ToPointer(), bmp->Width * 3);
                bmp->UnlockBits(bmpData);

                DetectAndDrawWhiteRectangle(mat);
            }
            else
            {
                request = dynamic_cast<HttpWebRequest^>(WebRequest::Create(url));
                response = dynamic_cast<HttpWebResponse^>(request->GetResponse());
                // Dispose of the response object to free up resources
                response->Close();
            }
        }
        catch (System::Exception^ ex) {
            timer1->Stop();
			timer1->Enabled = false;
            this->Invoke(gcnew Action<System::String^>(this, &MyForm::UpdateRunRobotButtonText), "Run Robot");
            MessageBox::Show("Error retrieving image: " + ex->Message);
        }
        return Task::CompletedTask;
    }

	private: void UpdateRunRobotButtonText(System::String^ text) {
		btnRunRobot->Text = text;
	}

	private: System::String^ GetConnectionText() {
		return cbxConnection->Text;
	}



	private: System::Void btnRunRobot_Click(System::Object^ sender, System::EventArgs^ e) {
		timer1->Enabled = true;
		int cmd;
		if (client != nullptr && client->Connected) {
			isRobotRunning = !isRobotRunning;
			if (isRobotRunning) {
				btnRunRobot->Text = L"🔳 Stop";
				btnRunRobot->ForeColor = System::Drawing::Color::Red;
				timer1->Enabled = true;
				timer1->Start();
				cmd = _RUNROBOT_;
				System::String^ url = "http://" + cbxConnection->Text + "/robot?cmd=" + cmd.ToString() +
					"&value1=" + iCenterX.ToString() + "&value2=" + iCenterY.ToString() +
					"&value3=" + iAngle.ToString() + "&value4=0&value5=0&value6=0";
				httpRequest = dynamic_cast<HttpWebRequest^>(WebRequest::Create(url));
				httpResponse = dynamic_cast<HttpWebResponse^>(httpRequest->GetResponse());
				// Dispose of the response object to free up resources
				httpResponse->Close();

			}
			else {
				btnRunRobot->Text = L"▶️ Start";
				btnRunRobot->ForeColor = System::Drawing::Color::Green;
				timer1->Stop();
				//timer1->Enabled = false;
				cmd = _STOPROBOT_;
				System::String^ url = "http://" + cbxConnection->Text + "/robot?cmd=" + cmd.ToString() +
					"&value1=0&value2=0&value3=0&value4=0&value5=0&value6=0";
				httpRequest = dynamic_cast<HttpWebRequest^>(WebRequest::Create(url));
				httpResponse = dynamic_cast<HttpWebResponse^>(httpRequest->GetResponse());
				// Dispose of the response object to free up resources
				httpResponse->Close();
			}
		}
	}
	private: System::Void cbxConnection_SelectedIndexChanged(System::Object^ sender, System::EventArgs^ e) {
	}
private: System::Void btnSavePos_Click(System::Object^ sender, System::EventArgs^ e) {
	int cmd = _SAVEPOS_;
	if (client != nullptr && client->Connected) {
		try {
			System::String^ url = "http://" + cbxConnection->Text + "/robot?cmd="+cmd.ToString() + "&value1 = " + iTheta1.ToString() +
				"&value2=" + iTheta2.ToString() + "&value3=" + iTheta3.ToString() +
				"&value4=" + iTheta4.ToString() + "&value5=" + iTheta5.ToString() +
				"&value6=" + iTheta6.ToString();
			httpRequest = dynamic_cast<HttpWebRequest^>(WebRequest::Create(url));
			httpResponse = dynamic_cast<HttpWebResponse^>(httpRequest->GetResponse());
			// Dispose of the response object to free up resources
			httpResponse->Close();
			MessageBox::Show("Position saved successfully", "Success", MessageBoxButtons::OK, MessageBoxIcon::Information);
		}
		catch (System::Exception^ ex) {
			MessageBox::Show("Error saving position: " + ex->Message, "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
		}
	}
}
private: System::Void btnClearPos_Click(System::Object^ sender, System::EventArgs^ e) {  
int cmd = _CLEARPOS_;  
if (client != nullptr && client->Connected) {  
	try {  
		System::String^ url = "http://" + cbxConnection->Text + "/robot?cmd=" + cmd.ToString() +  
			"&value1=0&value2=0&value3=0&value4=0&value5=0&value6=0";  
		httpRequest = dynamic_cast<HttpWebRequest^>(WebRequest::Create(url));  
		httpResponse = dynamic_cast<HttpWebResponse^>(httpRequest->GetResponse());  
		// Dispose of the response object to free up resources  
		httpResponse->Close();  
		MessageBox::Show("Position cleared successfully", "Success", MessageBoxButtons::OK, MessageBoxIcon::Information);  
	}  
	catch (System::Exception^ ex) {  
		MessageBox::Show("Error clearing position: " + ex->Message, "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);  
	}  
}  
}
private: System::Void btnClearPath_Click(System::Object^ sender, System::EventArgs^ e) {
	int cmd = _CLEARPATH_;
	if (client != nullptr && client->Connected) {
		try {
			System::String^ url = "http://" + cbxConnection->Text + "/robot?cmd=" + cmd.ToString() +
				"&value1=0&value2=0&value3=0&value4=0&value5=0&value6=0";
			httpRequest = dynamic_cast<HttpWebRequest^>(WebRequest::Create(url));
			httpResponse = dynamic_cast<HttpWebResponse^>(httpRequest->GetResponse());
			// Dispose of the response object to free up resources
			httpResponse->Close();
			MessageBox::Show("Path cleared successfully", "Success", MessageBoxButtons::OK, MessageBoxIcon::Information);
		}
		catch (System::Exception^ ex) {
			MessageBox::Show("Error clearing path: " + ex->Message, "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
		}
	}
}
};
}

