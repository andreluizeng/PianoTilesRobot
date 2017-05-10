
HANDLE          hPort;
DCB             COMDCB;
COMMTIMEOUTS    CommTimeouts;

#define TMsg 1
byte MsgRecPic[TMsg];


//---------------------------------------------------------------------------
void EscrevePorta (char *msg)
{
	DWORD i;

	BYTE x[TMsg];

	for (i = 0; i < TMsg; i++)
		x[i] = msg[i];

	WriteFile (hPort, &x, TMsg, &i,NULL);

}
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
int LePorta (void)
{
	DWORD i;
	int cont;
	BYTE x[TMsg];
	x[0] = 0x00;
	ReadFile (hPort, &x, TMsg, &i,NULL);

	// startbyte ok

	if (x[0] == 0x0a)
	{
/*		cont = 0;

		// enquanto nao chegar o stopbyte - continua o recebimento de dados
		while (x[cont] != '#')
		{
			MsgRecPic[cont - 1] = x[cont];
			cont++;
		}*/

		return true;
	}

	return false;
}
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// Abre a porta "Porta" com velocidade de 57600bps
// 8 bits de dados, 1 stopbit e sem paridade
DWORD AbrePorta (LPCWSTR Porta)
{
	hPort = CreateFile (Porta,  //Nome da porta
						GENERIC_READ | GENERIC_WRITE, // leitura e escrita
						0, // sem compartilhamento
					NULL, // ponteiro para atributo de seguranca
					OPEN_EXISTING, // como abrur a porta serial
			0, // atributos da porta
			NULL); // handle da porta para copia (NULL para com)

	if(hPort == INVALID_HANDLE_VALUE)
	{
			printf ("\nErro abrindo porta\n");
			return false;
	}

	// inicializa DCBLenght
	COMDCB.DCBlength = sizeof (DCB);

	// pega a informação default da configuração da porta
	GetCommState (hPort, &COMDCB);

	//configurações da estrutura DCB
	COMDCB.BaudRate = 115200;	// velocidade
	COMDCB.ByteSize	= 8;		// número de bits por mensagem
	COMDCB.Parity	= NOPARITY;	// tipo de paridade
	COMDCB.StopBits	= ONESTOPBIT; 	// número de stopbits

		// ou também pode ser usada a função BuildCommDCB
		// BuildCommDCB("57600,N,8,1", &COMDCB);

	// configura a porta de acordo com a estrutura DCB
	if (! SetCommState (hPort, &COMDCB))
	{
			printf ("\nErro abrindo porta\n");
			GetLastError ();
	}


	// definição de timeouts para leitura e escrita
	GetCommTimeouts (hPort, &CommTimeouts);
	CommTimeouts.ReadIntervalTimeout         = 2;
	CommTimeouts.ReadTotalTimeoutMultiplier  = 0;
	CommTimeouts.ReadTotalTimeoutConstant    = 2;
	CommTimeouts.WriteTotalTimeoutMultiplier = 5;
	CommTimeouts.WriteTotalTimeoutConstant   = 5;

	// configura os timeouts para todas as operacoes de leitura  e escrita
	if (! SetCommTimeouts (hPort, &CommTimeouts))
	{
		printf ("\nErro abrindo porta\n");
		GetLastError ();

		return FALSE;
	}

	return TRUE;
}


//---------------------------------------------------------------------------
// fecha a porta especificada
DWORD FechaPorta (void)
{
	CloseHandle (hPort);
		return TRUE;
}


//----------------------------------------------------------------------------
