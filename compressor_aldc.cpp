
//inclusiones.
#include "compressor_aldc.h"

//-------------------------------------------------------------------------------------
//m�todo constructor 1.
compressor_aldc::compressor_aldc()
{
	
}

//-------------------------------------------------------------------------------------
//m�todo constructor 2.
compressor_aldc::compressor_aldc(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size)
{
	//data settings.
	this->bits_resolution_data	= _bits_resolution_data;									//resoluci�n en bits de los datos.
	this->max_samples_amount	= _max_samples_amount;										//cantidad maxima de muestras a tomar.
	this->max_buffer_size		= _max_buffer_size;											//m�ximo tama�o del buffer de compresi�n de datos.

	//buffers.
	this->di_aldc				= new int[_max_samples_amount];								//diferencias entre xi y xi_1 para ALDC;
	this->ci_a					= new unsigned char[_max_buffer_size];						//buffer auxiliar para this->ci_a.
	this->ci_b					= new unsigned char[_max_buffer_size];						//buffer auxiliar para this->ci_b.
	this->ci_c					= new unsigned char[_max_buffer_size];						//buffer auxiliar para this->ci_c.
	this->ci_a_index_available	= 0;														//�ndice de buffer "this->ci_a" disponible para realizar concatenaci�n.
	this->ci_b_index_available	= 0;														//�ndice de buffer "this->ci_b" disponible para realizar concatenaci�n.
	this->ci_c_index_available	= 0;														//�ndice de buffer "this->ci_c" disponible para realizar concatenaci�n.
	this->ci_a_available_bits	= 8;														//cantidad de bits disponibles para realizar concatenaci�n en buffer "this->ci_a".
	this->ci_b_available_bits	= 8;														//cantidad de bits disponibles para realizar concatenaci�n en buffer "this->ci_b".
	this->ci_c_available_bits	= 8;														//cantidad de bits disponibles para realizar concatenaci�n en buffer "this->ci_c".

	//inicializaci�n.
	this->cleanBuffersALDC();
	this->iniTablesALDC();
}

//-------------------------------------------------------------------------------------
//m�todo destructor.
compressor_aldc::~compressor_aldc()
{

}

//-------------------------------------------------------------------------------------
//m�todo inicializaci�n.
void compressor_aldc::initialization(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size)
{
	//data settings.
	this->bits_resolution_data	= _bits_resolution_data;									//resoluci�n en bits de los datos.
	this->max_samples_amount	= _max_samples_amount;										//cantidad maxima de muestras a tomar.
	this->max_buffer_size		= _max_buffer_size;											//m�ximo tama�o del buffer de compresi�n de datos.

	//buffers.
	this->di_aldc				= new int[max_samples_amount];								//diferencias entre xi y xi_1 para ALDC;
	this->ci_a					= new unsigned char[_max_buffer_size];						//buffer auxiliar para this->ci_a.
	this->ci_b					= new unsigned char[_max_buffer_size];						//buffer auxiliar para this->ci_b.
	this->ci_c					= new unsigned char[_max_buffer_size];						//buffer auxiliar para this->ci_c.
	this->ci_a_index_available	= 0;														//�ndice de buffer "this->ci_a" disponible para realizar concatenaci�n.
	this->ci_b_index_available	= 0;														//�ndice de buffer "this->ci_b" disponible para realizar concatenaci�n.
	this->ci_c_index_available	= 0;														//�ndice de buffer "this->ci_c" disponible para realizar concatenaci�n.
	this->ci_a_available_bits	= 8;														//cantidad de bits disponibles para realizar concatenaci�n en buffer "this->ci_a".
	this->ci_b_available_bits	= 8;														//cantidad de bits disponibles para realizar concatenaci�n en buffer "this->ci_b".
	this->ci_c_available_bits	= 8;														//cantidad de bits disponibles para realizar concatenaci�n en buffer "this->ci_c".

	//inicializaci�n.
	this->cleanBuffersALDC();
	this->iniTablesALDC();
}

//-------------------------------------------------------------------------------------
//m�todo de compresi�n ALDC.
int compressor_aldc::compressorALDC(int *_data_in, int _samples_amount, int _bits_resolution_data, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits)
{
	//variables auxiliares.	
	int F		= 0;
	int	DR		= 0;	
	
	//seteo del primer elemento de "di" y de "F".
	this->di_aldc[0]	= _data_in[0] - pow(2, _bits_resolution_data - 1);
	F			= abs(this->di_aldc[0]);

	//bucle para obtenci�n de los "di" y de "F".
	for (int i1 = 1; i1 < _samples_amount; i1++)
	{
		//obtenci�n de los "di".
		this->di_aldc[i1] = _data_in[i1] - _data_in[i1 - 1];
		
		//c�lculo de "F".
		F			+= abs(this->di_aldc[i1]);
	}

	//obtenci�n de la regi�n de desici�n.
	DR = this->getDR(F, _samples_amount);
	
	//si la regi�n de desici�n es "II", entonces.
	if (DR == 2)
	{
		//inicializaci�n de c�digo..
		this->joinBits(0, 1, _stream_com_data, _stream_index_available, _stream_available_bits);

		//obtenci�n de c�digo.
		this->IITableALECEncoder(this->di_aldc, _samples_amount, _stream_com_data, _stream_index_available, _stream_available_bits);
	}

	//si la regi�n de desici�n es "III", entonces.
	else if (DR == 3)
	{
		//inicializaci�n de c�digo..
		this->joinBits(1, 1, _stream_com_data, _stream_index_available, _stream_available_bits);

		//obtenci�n de c�digo.
		this->IIITableALECEncoder(this->di_aldc, _samples_amount, _stream_com_data, _stream_index_available, _stream_available_bits);
	}

	//retorno de la cantidad de bytes resultantes de la compresi�n.
	if (_stream_available_bits == 8)	return _stream_index_available;
	else								return _stream_index_available + 1;
}

//-------------------------------------------------------------------------------------
//m�todo de descompresi�n ALDC.
int compressor_aldc::decompressorALDC(unsigned char *_stream_com_data, int _samples_amount, int *_data_out)
{
	//variables auxiliares.	
	int	DR						= 0;
	int id_table				= 0;
	int i1						= 0;
	int hi						= 0;
	int bi						= 0;
	int bi_aux					= 0;
	int di_index				= 0;
	int	bit_position			= 0;
	int bits_counter			= 0;
	int	remaining_bits_amount	= 0;
	int temp					= 0;
	int temp_aux				= 0;

	//obtenci�n de "DR".
	DR	= _stream_com_data[0] >> 7;

	//obtenci�n de "id_table".
	if (DR == 0)
	{
		//seteo de "id_table".
		id_table = (_stream_com_data[0] >> 6) & 1;

		//seteo de posici�n del bit a leer.
		bit_position = 5;
	}
	else
	{
		//obtenci�n del primer bit.
		temp = (_stream_com_data[0] >> 6) & 1;
		
		//si el primer bit es cero, entonces.
		if (temp == 0)
		{
			//seteo de "id_table".
			id_table = 2;

			//seteo de posici�n del bit a leer.
			bit_position = 5;
		}
		else
		{
			//obtenci�n del segundo bit.
			temp = (_stream_com_data[0] >> 5) & 1;

			if (temp == 0)	id_table = 0;
			else			id_table = 1;

			//seteo de posici�n del bit a leer.
			bit_position = 4;
		}
	}

	//bucle para obtener todos los "di".
	do
	{
		//bucle para obtener "bi".
		do
		{
			//obtenci�n de "hi".
			if ((_stream_com_data[i1] & (1 << bit_position)) == 0)	hi = hi << 1;
			else													hi = (hi << 1) | 1;

			//decremento de "bit_position".
			--bit_position;

			//si "bit_position" alcanza su m�nimo valor, entonces.
			if (bit_position == -1)
			{
				//actualizaciones.
				bit_position = 7;
				++i1;
			}

			//incremento del contador de bits.
			++bits_counter;

			//obtenci�n de "bi".
			bi = this->getBi(hi, bits_counter, id_table);

			//si se ha obtenido "bi", entonces.
			if (bi != -1) break;

		} while (true);

		//reset de variables.
		bits_counter	= 0;
		hi				= 0;
		temp			= 0;
		temp_aux		= 0;
		bi_aux			= bi;

		//si "bi" es distinto de cero, entonces.
		if (bi != 0)
		{
			//bucle para obtener los primeros bits de "di".
			do
			{
				//obtenci�n de bits remanentes.
				remaining_bits_amount = bi_aux - (bit_position + 1);

				//si "di", no est� totalmente en un byte, entonces.
				if (remaining_bits_amount > 0)
				{
					//obtenci�n de los primeros bits de "di".
					temp_aux = _stream_com_data[i1] & ((1 << (bi_aux - remaining_bits_amount)) - 1);

					//seteo de los primeros bits de "di".
					this->joinBits(temp_aux, bi_aux - remaining_bits_amount, temp);

					//actualizaciones.
					bit_position			= 7;
					bi_aux					-= bi_aux - remaining_bits_amount;
					remaining_bits_amount	= 0;
					++i1;
				}

				//si "di", est� totalmente en un byte, entonces.
				else break;				

			} while (true);

			//lectura de c�digo.
			temp = (temp << bi_aux) | ((_stream_com_data[i1] >> (bit_position - bi_aux + 1)) & ((1 << bi_aux) - 1));
			
			//seteo de "di".
			if (this->getBitsAmount(temp) == bi)	this->di_aldc[di_index] = temp;
			else									this->di_aldc[di_index] = temp - pow(2, bi) + 1;
			
			//decremento de "bit_position".
			bit_position -= bi_aux;

			//si "bit_position" alcanza su m�nimo valor, entonces.
			if (bit_position == -1)
			{
				//actualizaciones.
				bit_position = 7;
				++i1;
			}		
		}

		//si "bi" no es distinto de cero, entonces.
		else
		{
			//obtenci�n de "di".
			this->di_aldc[di_index] = 0;
		}
		
		//incremento del �ndice de "di".
		++di_index;	

		//si sobrepasa el n�mero de datos, entonces salir.
		if ((di_index + 1) > _samples_amount) break;

	} while (true);

	//obtenci�n de "data_out[0]".
	_data_out[0] = pow(2, this->bits_resolution_data - 1) + this->di_aldc[0];

	//bucle para obtener datos de salida.
	for (int i2 = 1; i2 < _samples_amount; i2++)
	{
		//obtenci�n de "data_out[i2]".
		_data_out[i2] = _data_out[i2 - 1] + this->di_aldc[i2];
	}

	//limpieza de buffers.
	this->cleanBuffersALDC();

	//retorno de la cantidad de bytes resultantes de la descompresi�n.
	return (_samples_amount * ceil(float(this->bits_resolution_data) / 8.f));
}

//-------------------------------------------------------------------------------------
//m�todo para codificar con tabla 2.
void compressor_aldc::IITableALECEncoder(int *_di, int _samples_amount, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits)
{
	//buffer de salida auxiliar.
	int	busy_bits_ci_a	= 0;
	int	busy_bits_ci_b	= 0;
	int	id				= 0;
	
	//concatenaci�n de "id" en "this->ci_a" y "this->ci_b".
	this->joinBits(0, 1, this->ci_a, this->ci_a_index_available, this->ci_a_available_bits);
	this->joinBits(1, 1, this->ci_b, this->ci_b_index_available, this->ci_b_available_bits);
	
	//bucle para obtener "ci" con tabla "A" y "B".
	for (int i1 = 0; i1 < _samples_amount; i1++)
	{
		//obtenci�n de "ci".
		this->encode(_di[i1], 0, this->ci_a, busy_bits_ci_a, this->ci_a_index_available, this->ci_a_available_bits);
		this->encode(_di[i1], 1, this->ci_b, busy_bits_ci_b, this->ci_b_index_available, this->ci_b_available_bits);
	}

	//comparaci�n de la mayor compresi�n.
	if (busy_bits_ci_a <= busy_bits_ci_b)	id = 0;
	else									id = 1;

	//concatenaci�n de "ci" ganador.
	if (id == 0)
	{
		//bucle para concatenar bits a "_stream_com_data".
		for (int i1 = 0; i1 < this->ci_a_index_available; i1++)
		{
			//concatenaci�n de bits en "_stream_com_data".
			this->joinBits(this->ci_a[i1], 8, _stream_com_data, _stream_index_available, _stream_available_bits);
		}

		//obtenci�n del �ltimo conjunto de bits.
		if (this->ci_a_available_bits < 8)
		{
			//nits restantes a concatenar.
			this->ci_a[this->ci_a_index_available] = this->ci_a[this->ci_a_index_available] >> this->ci_a_available_bits;

			//concatenaci�n de bits en "_stream_com_data".
			this->joinBits(this->ci_a[this->ci_a_index_available], 8 - this->ci_a_available_bits, _stream_com_data, _stream_index_available, _stream_available_bits);
		}
	}
	else
	{
		//bucle para concatenar bits a "_stream_com_data".
		for (int i1 = 0; i1 < this->ci_b_index_available; i1++)
		{
			//concatenaci�n de bits en "_stream_com_data".
			this->joinBits(this->ci_b[i1], 8, _stream_com_data, _stream_index_available, _stream_available_bits);
		}

		//obtenci�n del �ltimo conjunto de bits.
		if (this->ci_b_available_bits < 8)
		{
			//nits restantes a concatenar.
			this->ci_b[this->ci_b_index_available] = this->ci_b[this->ci_b_index_available] >> this->ci_b_available_bits;

			//concatenaci�n de bits en "_stream_com_data".
			this->joinBits(this->ci_b[this->ci_b_index_available], 8 - this->ci_b_available_bits, _stream_com_data, _stream_index_available, _stream_available_bits);
		}
	}	
}

//-------------------------------------------------------------------------------------
//m�todo para codificar con tabla 3.
void compressor_aldc::IIITableALECEncoder(int *_di, int _samples_amount, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits)
{
	//buffer de salida auxiliar.	
	int	busy_bits_ci_a	= 0;
	int	busy_bits_ci_b	= 0;
	int	busy_bits_ci_c	= 0;
	int	id				= 0;

	//concatenaci�n de "id" en "this->ci_a", "this->ci_b" y "this->ci_c".
	this->joinBits(10, 2, this->ci_a, this->ci_a_index_available, this->ci_a_available_bits);
	this->joinBits(11, 2, this->ci_b, this->ci_b_index_available, this->ci_b_available_bits);
	this->joinBits(0, 1, this->ci_c, this->ci_c_index_available, this->ci_c_available_bits);

	//bucle para obtener "ci" con tabla "A", "B" y "C".
	for (int i1 = 0; i1 < _samples_amount; i1++)
	{
		//obtenci�n de "ci".
		this->encode(_di[i1], 0, this->ci_a, busy_bits_ci_a, this->ci_a_index_available, this->ci_a_available_bits);
		this->encode(_di[i1], 1, this->ci_b, busy_bits_ci_b, this->ci_b_index_available, this->ci_b_available_bits);
		this->encode(_di[i1], 2, this->ci_c, busy_bits_ci_c, this->ci_c_index_available, this->ci_c_available_bits);
	}

	//comparaci�n de la mayor compresi�n.
	if		(busy_bits_ci_a <= minor(busy_bits_ci_b, busy_bits_ci_c))	id = 10;
	else if (busy_bits_ci_b <= minor(busy_bits_ci_a, busy_bits_ci_c))	id = 11;
	else if (busy_bits_ci_c <= minor(busy_bits_ci_a, busy_bits_ci_b))	id = 0;
	
	//concatenaci�n de "ci" ganador.
	if (id == 10)
	{
		//bucle para concatenar bits a "_stream_com_data".
		for (int i1 = 0; i1 < this->ci_a_index_available; i1++)
		{
			//concatenaci�n de bits en "_stream_com_data".
			this->joinBits(this->ci_a[i1], 8, _stream_com_data, _stream_index_available, _stream_available_bits);
		}

		//obtenci�n del �ltimo conjunto de bits.
		if (this->ci_a_available_bits < 8)
		{
			//nits restantes a concatenar.
			this->ci_a[this->ci_a_index_available] = this->ci_a[this->ci_a_index_available] >> this->ci_a_available_bits;

			//concatenaci�n de bits en "_stream_com_data".
			this->joinBits(this->ci_a[this->ci_a_index_available], 8 - this->ci_a_available_bits, _stream_com_data, _stream_index_available, _stream_available_bits);
		}
	}
	else if(id == 11)
	{
		//bucle para concatenar bits a "_stream_com_data".
		for (int i1 = 0; i1 < this->ci_b_index_available; i1++)
		{
			//concatenaci�n de bits en "_stream_com_data".
			this->joinBits(this->ci_b[i1], 8, _stream_com_data, _stream_index_available, _stream_available_bits);
		}

		//obtenci�n del �ltimo conjunto de bits.
		if (this->ci_b_available_bits < 8)
		{
			//nits restantes a concatenar.
			this->ci_b[this->ci_b_index_available] = this->ci_b[this->ci_b_index_available] >> this->ci_b_available_bits;

			//concatenaci�n de bits en "_stream_com_data".
			this->joinBits(this->ci_b[this->ci_b_index_available], 8 - this->ci_b_available_bits, _stream_com_data, _stream_index_available, _stream_available_bits);
		}
	}	
	else if (id == 0)
	{
		//bucle para concatenar bits a "_stream_com_data".
		for (int i1 = 0; i1 < this->ci_c_index_available; i1++)
		{
			//concatenaci�n de bits en "_stream_com_data".
			this->joinBits(this->ci_c[i1], 8, _stream_com_data, _stream_index_available, _stream_available_bits);
		}

		//obtenci�n del �ltimo conjunto de bits.
		if (this->ci_c_available_bits < 8)
		{
			//nits restantes a concatenar.
			this->ci_c[this->ci_c_index_available] = this->ci_c[this->ci_c_index_available] >> this->ci_c_available_bits;

			//concatenaci�n de bits en "_stream_com_data".
			this->joinBits(this->ci_c[this->ci_c_index_available], 8 - this->ci_c_available_bits, _stream_com_data, _stream_index_available, _stream_available_bits);
		}
	}
}

//-------------------------------------------------------------------------------------
//m�todo para codificar "di".
void compressor_aldc::encode(int _di, int _table, unsigned char *_ci, int &_busy_bits_ci, int &_ci_index_available, int &_ci_available_bits)
{
	//variables auxiliares.
	int bi				= 0;
	int hi				= 0;
	int li				= 0;
	int index			= 0;
	int busy_bits_hi	= 0;
	int busy_bits_li	= 0;

	//seteo de "bi".
	if	(_di == 0)	bi = 0;
	else			bi = this->getLog2Di(abs(_di));
	
	//obtenci�n de "hi".
	this->getHi(bi, _table, hi, busy_bits_hi);
	
	//construcci�n de "ci".
	if (bi == 0)
	{
		//seteo de "ci".
		this->joinBits(hi, busy_bits_hi, _ci, _ci_index_available, _ci_available_bits);

		//seteo de la cantidad de bits utilizados para "ci".
		_busy_bits_ci	+= busy_bits_hi;
	}
	else
	{
		//obtenci�n de "index".
		index = getIndex(_di, bi);
		
		//seteo de la cantidad de bits utilizados por "li".
		busy_bits_li = bi;

		//seteo de "li".
		this->joinBits(index, busy_bits_li, li);
		
		//seteo de "ci".
		this->joinBits(hi, busy_bits_hi, _ci, _ci_index_available, _ci_available_bits);
		this->joinBits(li, busy_bits_li, _ci, _ci_index_available, _ci_available_bits);
		
		//seteo de la cantidad de bits utilizados para "ci".
		_busy_bits_ci += busy_bits_hi + busy_bits_li;
	}
}

//-------------------------------------------------------------------------------------
//m�todo para inicializar tablas.
void compressor_aldc::iniTablesALDC()
{
	//incializaci�n de valores de tabla 1.
	this->tables_aldc[0][0][0]	= 0;
	this->tables_aldc[0][1][0]	= 1;
	this->tables_aldc[0][2][0]	= 3;
	this->tables_aldc[0][3][0]	= 5;
	this->tables_aldc[0][4][0]	= 9;
	this->tables_aldc[0][5][0]	= 17;
	this->tables_aldc[0][6][0]	= 33;
	this->tables_aldc[0][7][0]	= 65;
	this->tables_aldc[0][8][0]	= 129;
	this->tables_aldc[0][9][0]	= 512;
	this->tables_aldc[0][10][0]	= 1026;
	this->tables_aldc[0][11][0]	= 1027;
	this->tables_aldc[0][12][0]	= 1028;
	this->tables_aldc[0][13][0]	= 1029;
	this->tables_aldc[0][14][0]	= 1030;

	//incializaci�n de cantidad de bits por valores de tabla 1.
	this->tables_aldc[0][0][1]	= 2;
	this->tables_aldc[0][1][1]	= 2;
	this->tables_aldc[0][2][1]	= 2;
	this->tables_aldc[0][3][1]	= 3;
	this->tables_aldc[0][4][1]	= 4;
	this->tables_aldc[0][5][1]	= 5;
	this->tables_aldc[0][6][1]	= 6;
	this->tables_aldc[0][7][1]	= 7;
	this->tables_aldc[0][8][1]	= 8;
	this->tables_aldc[0][9][1]	= 10;
	this->tables_aldc[0][10][1]	= 11;
	this->tables_aldc[0][11][1]	= 11;
	this->tables_aldc[0][12][1]	= 11;
	this->tables_aldc[0][13][1]	= 11;
	this->tables_aldc[0][14][1]	= 11;

	//incializaci�n de valores de tabla 2.
	this->tables_aldc[1][0][0]	= 111;
	this->tables_aldc[1][1][0]	= 26;
	this->tables_aldc[1][2][0]	= 12;
	this->tables_aldc[1][3][0]	= 3;
	this->tables_aldc[1][4][0]	= 7;
	this->tables_aldc[1][5][0]	= 2;
	this->tables_aldc[1][6][0]	= 0;
	this->tables_aldc[1][7][0]	= 2;
	this->tables_aldc[1][8][0]	= 54;
	this->tables_aldc[1][9][0]	= 443;
	this->tables_aldc[1][10][0]	= 441;
	this->tables_aldc[1][11][0]	= 885;
	this->tables_aldc[1][12][0]	= 884;
	this->tables_aldc[1][13][0]	= 880;
	this->tables_aldc[1][14][0]	= 1763;

	//incializaci�n de cantidad de bits por valores de tabla 2.
	this->tables_aldc[1][0][1]	= 7;
	this->tables_aldc[1][1][1]	= 5;
	this->tables_aldc[1][2][1]	= 4;
	this->tables_aldc[1][3][1]	= 3;
	this->tables_aldc[1][4][1]	= 3;
	this->tables_aldc[1][5][1]	= 2;
	this->tables_aldc[1][6][1]	= 2;
	this->tables_aldc[1][7][1]	= 3;
	this->tables_aldc[1][8][1]	= 6;
	this->tables_aldc[1][9][1]	= 9;
	this->tables_aldc[1][10][1]	= 9;
	this->tables_aldc[1][11][1]	= 10;
	this->tables_aldc[1][12][1]	= 10;
	this->tables_aldc[1][13][1]	= 10;
	this->tables_aldc[1][14][1]	= 11;

	//incializaci�n de valores de tabla 3.
	this->tables_aldc[2][0][0]	= 9;
	this->tables_aldc[2][1][0]	= 5;
	this->tables_aldc[2][2][0]	= 0;
	this->tables_aldc[2][3][0]	= 1;
	this->tables_aldc[2][4][0]	= 3;
	this->tables_aldc[2][5][0]	= 17;
	this->tables_aldc[2][6][0]	= 33;
	this->tables_aldc[2][7][0]	= 65;
	this->tables_aldc[2][8][0]	= 129;
	this->tables_aldc[2][9][0]	= 512;
	this->tables_aldc[2][10][0]	= 1026;
	this->tables_aldc[2][11][0]	= 1027;
	this->tables_aldc[2][12][0]	= 1028;
	this->tables_aldc[2][13][0]	= 1029;
	this->tables_aldc[2][14][0]	= 1030;

	//incializaci�n de cantidad de bits por valores de tabla 3.
	this->tables_aldc[2][0][1]	= 4;
	this->tables_aldc[2][1][1]	= 3;
	this->tables_aldc[2][2][1]	= 2;
	this->tables_aldc[2][3][1]	= 2;
	this->tables_aldc[2][4][1]	= 2;
	this->tables_aldc[2][5][1]	= 5;
	this->tables_aldc[2][6][1]	= 6;
	this->tables_aldc[2][7][1]	= 7;
	this->tables_aldc[2][8][1]	= 8;
	this->tables_aldc[2][9][1]	= 10;
	this->tables_aldc[2][10][1]	= 11;
	this->tables_aldc[2][11][1]	= 11;
	this->tables_aldc[2][12][1]	= 11;
	this->tables_aldc[2][13][1]	= 11;
	this->tables_aldc[2][14][1]	= 11;
}

//-------------------------------------------------------------------------------------
//m�todo para obtener "DR".
int compressor_aldc::getDR(int _F, int _samples_amount)
{
	//obtenci�n de la regi�n de decisi�n.
	if (_F <= 3 * _samples_amount)								return 2;
	if (_F > 3 * _samples_amount && _F <= 12 * _samples_amount)	return 3;
	if (_F > 12 * _samples_amount)								return 2;
	
	//retorno por defecto.
	return -1;
}

//-------------------------------------------------------------------------------------
//m�todo para obtener "hi" en tabla.
void compressor_aldc::getHi(int _bi, int _table, int &_hi, int &_busy_bits_hi)
{
	//obtenci�n de "bi" seg�n tabla.
	_hi				= this->tables_aldc[_table][_bi][0];
	_busy_bits_hi	= this->tables_aldc[_table][_bi][1];
}

//-------------------------------------------------------------------------------------
//m�todo para obtener "bi" en tabla.
int compressor_aldc::getBi(int _hi, int _busy_bits_hi, int _table)
{
	//bucle para cotejar "_hi" en tabla.
	for (int i1 = 0; i1 < 15; i1++)
	{
		//si existe en tabla, entonces.
		if (_hi == this->tables_aldc[_table][i1][0] && _busy_bits_hi == this->tables_aldc[_table][i1][1]) return i1;
	}

	//si no existe en tabla, entonces.
	return -1;
}

//-------------------------------------------------------------------------------------
//m�todo para obtener "index".
int compressor_aldc::getIndex(int _di, int _bi)
{
	//obtenci�n de "index".
	if (_di >= 0) return _di;
	else		  return ((pow(2, _bi) - 1) + _di);
}

//-------------------------------------------------------------------------------------
//m�todo para limpiar buffers.
void compressor_aldc::cleanBuffersALDC()
{
	//limpieza de buffer auxiliares.
	this->ci_a_index_available	= 0;
	this->ci_b_index_available	= 0;
	this->ci_c_index_available	= 0;
	this->ci_a_available_bits	= 8;
	this->ci_b_available_bits	= 8;
	this->ci_c_available_bits	= 8;

	//bucle para limpiar buffers.
	for (int i1 = 0; i1 < this->max_buffer_size; i1++)
	{
		//limpieza.
		this->ci_a[i1]			= 0;
		this->ci_b[i1]			= 0;
		this->ci_c[i1]			= 0;
	}

	//bucle para limpiar buffers.
	for (int i1 = 0; i1 < this->max_samples_amount; i1++)
	{
		//limpieza.
		this->di_aldc[i1]		= 0;
	}
}
