
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//inclusiones.
#include "compressor_lec.h"

//-------------------------------------------------------------------------------------
//método constructor 1.
compressor_lec::compressor_lec()
{
	
}

//-------------------------------------------------------------------------------------
//método constructor 2.
compressor_lec::compressor_lec(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size)
{
	//data settings.
	this->bits_resolution_data	= _bits_resolution_data;									//resolución en bits de los datos.
	this->max_samples_amount	= _max_samples_amount;										//cantidad maxima de muestras a tomar.
	this->max_buffer_size		= _max_buffer_size;											//máximo tamaño del buffer de compresión de datos.
	
	//buffers.
	this->di_lec				= new int[max_samples_amount];								//diferencias entre xi y xi_1 para LEC.

	//inicialización.
	this->cleanBuffersLEC();
	this->iniTableLEC();
}

//-------------------------------------------------------------------------------------
//método constructor 3.
compressor_lec::compressor_lec(int _max_samples_amount, int _max_buffer_size)
{
	//data settings.
	this->max_samples_amount	= _max_samples_amount;										//cantidad maxima de muestras a tomar.
	this->max_buffer_size		= _max_buffer_size;											//máximo tamaño del buffer de compresión de datos.
	
	//buffers.
	this->di_lec				= new int[max_samples_amount];								//diferencias entre xi y xi_1 para LEC.

	//inicialización.
	this->cleanBuffersLEC();
	this->iniTableLEC();
}

//-------------------------------------------------------------------------------------
//método destructor.
compressor_lec::~compressor_lec()
{

}

//-------------------------------------------------------------------------------------
//método inicialización 1.
void compressor_lec::initialization(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size)
{
	//data settings.
	this->bits_resolution_data	= _bits_resolution_data;									//resolución en bits de los datos.
	this->max_samples_amount	= _max_samples_amount;										//cantidad maxima de muestras a tomar.
	this->max_buffer_size		= _max_buffer_size;											//máximo tamaño del buffer de compresión de datos.
	
	//buffers.
	this->di_lec				= new int[max_samples_amount];								//diferencias entre xi y xi_1 para LEC.

	//inicialización.
	this->cleanBuffersLEC();
	this->iniTableLEC();
}

//-------------------------------------------------------------------------------------
//método inicialización 2.
void compressor_lec::initialization(int _max_samples_amount, int _max_buffer_size)
{
	//data settings.
	this->max_samples_amount	= _max_samples_amount;										//cantidad maxima de muestras a tomar.
	this->max_buffer_size		= _max_buffer_size;											//máximo tamaño del buffer de compresión de datos.
	
	//buffers.
	this->di_lec				= new int[max_samples_amount];								//diferencias entre xi y xi_1 para LEC.

	//inicialización.
	this->cleanBuffersLEC();
	this->iniTableLEC();
}

//-------------------------------------------------------------------------------------
//método de compresión LEC.
int compressor_lec::compressorLEC(int *_data_in, int _samples_amount, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits)
{
	//variables auxiliares.
	int	ni				= 0;
	int	si				= 0;
	int	ai				= 0;
	int busy_bits_si	= 0;	

	//bucle para comprimir cada muestra.
	for (int i1 = 0; i1 < _samples_amount; i1++)
	{
		//seteo del primer elemento de "di".
		if(i1 == 0)	this->di_lec[i1]	= _data_in[i1];
		else		this->di_lec[i1]	= _data_in[i1] - _data_in[i1 - 1];		
		
		//codificación.
		if (this->di_lec[i1] == 0)	ni = 0;
		else						ni = this->getLog2Di(abs(this->di_lec[i1]));

		//seteo de si.
		this->getSi(ni, si, busy_bits_si);

		//construcción de "bsi".
		if (ni == 0)
		{
			//set si to _stream_com_data.
			this->joinBits(si, busy_bits_si, _stream_com_data, _stream_index_available, _stream_available_bits);
		}
		else
		{
			//si la diferencia es positiva, entonces.
			if (this->di_lec[i1] > 0)
			{
				//seteo de los "ni" bits menos significativos de "di".
				ai = this->di_lec[i1] & ((1 << ni) - 1);
			}
			else
			{
				//seteo de los "ni" bits menos significativos de "di -1".
				ai = (this->di_lec[i1] - 1) & ((1 << ni) - 1);
			}

			//set si to _stream_com_data.
			this->joinBits(si, busy_bits_si, _stream_com_data, _stream_index_available, _stream_available_bits);

			//set ai to _stream_com_data.
			this->joinBits(ai, ni, _stream_com_data, _stream_index_available, _stream_available_bits);
		}
	}

	//retorno de la cantidad de bytes resultantes de la compresión.
	if (_stream_available_bits == 8)	return _stream_index_available;
	else								return _stream_index_available + 1;
}

//-------------------------------------------------------------------------------------
//método de compresión LEC para flujo de datos.
int compressor_lec::compressorLEC(int _data_in_p, int _data_in, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits)
{
	//variables auxiliares.
	int	ni				= 0;
	int	si				= 0;
	int	ai				= 0;
	int busy_bits_si	= 0;	

	//seteo de "di".
	this->di_lec[0]		= _data_in - _data_in_p;

	//codificación.
	if (this->di_lec[0] == 0)	ni = 0;
	else						ni = this->getLog2Di(abs(this->di_lec[0]));

	//seteo de si.
	this->getSi(ni, si, busy_bits_si);

	//construcción de "bsi".
	if (ni == 0)
	{
		//set si to _stream_com_data.
		this->joinBits(si, busy_bits_si, _stream_com_data, _stream_index_available, _stream_available_bits);
	}
	else
	{
		//si la diferencia es positiva, entonces.   
		if (this->di_lec[0] > 0)
		{
			//seteo de los "ni" bits menos significativos de "di".
			ai = this->di_lec[0] & ((1 << ni) - 1);
		}
		else
		{
			//seteo de los "ni" bits menos significativos de "di -1".
			ai = (this->di_lec[0] - 1) & ((1 << ni) - 1);
		}

		//set si to _stream_com_data.
		this->joinBits(si, busy_bits_si, _stream_com_data, _stream_index_available, _stream_available_bits);

		//set ai to _stream_com_data.
		this->joinBits(ai, ni, _stream_com_data, _stream_index_available, _stream_available_bits);
	}

	//retorno de la cantidad de bytes resultantes de la compresión.
	if (_stream_available_bits == 8)	return _stream_index_available;
	else								return _stream_index_available + 1;
}

//-------------------------------------------------------------------------------------
//método de descompresión LEC 1 para un flujo de varios tipos de datos.
int compressor_lec::decompressorLEC(unsigned char *_stream_com_data, int *_data_in_p, int *_remaining_data_types, int _remaining_data_type_amount, int _data_types_amount, int _base_samples_amount, int *_samples_amount, int *_bits_resolution_data, int **_stream_decom_data)
{
	//índice para el tipo de dato en muestras base.
	int data_type_index					= 0;

	//índice para buffer de datos remanentes.
	int remaining_data_types_index		= 0;

	//contador de datos remanentes léidos.
	int remaining_data_types_counter	= 0;

	//bytes resultantes de la descompresión.
	int decompress_size					= 0;

	//contador de muestras.
	int base_samples_counter			= 0;

	//variables auxiliares para la descompresión.
	int i1								= 0;
	int si								= 0;
	int ni								= 0;
	int ni_aux							= 0;
	int	bit_position					= 7;
	int bits_counter					= 0;			
	int	remaining_bits_amount			= 0;
	int temp							= 0;
	int temp_aux						= 0;

	//bucle para descomprimir datos.  
	do
	{
		//bucle para obtener "ni".
		do
		{
			//obtención de "si".
			if ((_stream_com_data[i1] & (1 << bit_position)) == 0)	si = si << 1;
			else													si = (si << 1) | 1;
			
			//decremento de "bit_position".
			--bit_position;

			//si "bit_position" alcanza su mínimo valor, entonces.
			if (bit_position == -1)
			{
				//actualizaciones.
				bit_position = 7;
				++i1;
			}
			
			//incremento del contador de bits.
			++bits_counter;
			
			//obtención de "ni".
			ni = this->getNi(si, bits_counter);
			
			//si se ha obtenido "bi", entonces.
			if (ni != -1) break;

		} while (true);
		
		//reset de variables.
		bits_counter	= 0;
		si				= 0;
		temp			= 0;
		temp_aux		= 0;
		ni_aux			= ni;
		
		//si "ni" es distinto de cero, entonces.
		if (ni != 0)
		{
			//bucle para obtener los primeros bits de "di".
			do
			{
				//obtención de bits remanentes.
				remaining_bits_amount = ni_aux - (bit_position + 1);

				//si "di", no est� totalmente en un byte, entonces.
				if (remaining_bits_amount > 0)
				{
					//obtención de los primeros bits de "di".
					temp_aux = _stream_com_data[i1] & ((1 << (ni_aux - remaining_bits_amount)) - 1);
					
					//seteo de los primeros bits de "di".
					this->joinBits(temp_aux, ni_aux - remaining_bits_amount, temp);
					
					//actualizaciones.
					bit_position			= 7;
					ni_aux					-= ni_aux - remaining_bits_amount;
					remaining_bits_amount	= 0;
					++i1;
				}

				//si "di", est� totalmente en un byte, entonces.
				else break;				

			} while (true);
			
			//lectura de código.
			temp = (temp << ni_aux) | ((_stream_com_data[i1] >> (bit_position - ni_aux + 1)) & ((1 << ni_aux) - 1));
			
			//seteo de "di".
			if (this->getBitsAmount(temp) == ni)	this->di_lec[0] = temp;
			else									this->di_lec[0] = temp - int(pow(2, ni)) + 1;
			
			//decremento de "bit_position".
			bit_position -= ni_aux;

			//si "bit_position" alcanza su mínimo valor, entonces.
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
			//obtención de "di".
			this->di_lec[0] = 0;
		}	

		//mientras hayan posiciones de datos remanentes por leer, entonces.
		if (remaining_data_types_counter < _remaining_data_type_amount)
		{
			//bucle para leer si hay datos remanentes.
			for (int ix = remaining_data_types_index; ix < _data_types_amount; ix++)
			{
				//incremento del índice de datos remantenes.
				++remaining_data_types_index;

				//si existe un dato remanente de "i1", entonces.
				if (_remaining_data_types[ix] != 20)
				{
					//incremento del contador de datos remantenes.
					++remaining_data_types_counter;

					//obtención del primer dato descomprimido del tipo de dato "i1".
					_stream_decom_data[ix][0]		= _data_in_p[ix] + this->di_lec[0];

					//incremento de la cantidad de muestras de un tipo de datos.
					++_samples_amount[ix];

					//salida del bucle.
					break;
				}									
			}
		}

		else
		{
			//obtención del primer dato del tipo de dato.
			_stream_decom_data[data_type_index][_samples_amount[data_type_index]]		= _stream_decom_data[data_type_index][_samples_amount[data_type_index] - 1] + this->di_lec[0];

			//incremento de la cantidad de muestras de un tipo de datos.
			++_samples_amount[data_type_index];

			//incremento del índice del tipo de dato.
			++data_type_index;

			//si han pasado todos los tipos, entonces.
			if (data_type_index > (_data_types_amount - 1))
			{
				//incremento del contador de muestras bases.
				++base_samples_counter;

				//reset del índice de datos base.
				data_type_index = 0;
			}

			//si sobrepasa el número de datos base, entonces salir.
			if (base_samples_counter > _base_samples_amount) break;
		}		
			
		//limpieza de buffers.
		this->cleanBuffersLEC();

	} while (true);

	//bucle para obtener la cantidad de bytes resultantes.
	for (int i1 = 0; i1 < _data_types_amount; i1++)
	{
		//cálculo de bytes obtenidos de la descompresión.
		decompress_size += int(_samples_amount[i1] * ceil(float(_bits_resolution_data[i1]) / 8.f));
	}	

	//retorno de la cantidad de bytes resultantes de la descompresión.
	return decompress_size;
}

//-------------------------------------------------------------------------------------
//método de descompresión LEC 2 para varios tipos de datos ordenados por tipo.
int compressor_lec::decompressorLEC(unsigned char *_stream_com_data, int _data_types_amount, int *_samples_amount, int *_bits_resolution_data, int **_stream_decom_data)
{
	//índice para el tipo de dato.
	int data_type_index			= 0;

	//bytes resultantes de la descompresión.
	int decompress_size			= 0;

	//variables auxiliares para la descompresión.
	int i1						= 0;
	int si						= 0;
	int ni						= 0;
	int ni_aux					= 0;
	int di_index				= 0;
	int	bit_position			= 7;
	int bits_counter			= 0;			
	int	remaining_bits_amount	= 0;
	int temp					= 0;
	int temp_aux				= 0;

	//bucle para descomprimir datos.  
	do
	{
		//bucle para obtener "ni".
		do
		{
			//obtención de "si".
			if ((_stream_com_data[i1] & (1 << bit_position)) == 0)	si = si << 1;
			else													si = (si << 1) | 1;
			
			//decremento de "bit_position".
			--bit_position;

			//si "bit_position" alcanza su mínimo valor, entonces.
			if (bit_position == -1)
			{
				//actualizaciones.
				bit_position = 7;
				++i1;
			}
			
			//incremento del contador de bits.
			++bits_counter;
			
			//obtención de "ni".
			ni = this->getNi(si, bits_counter);
			
			//si se ha obtenido "bi", entonces.
			if (ni != -1) break;

		} while (true);
		
		//reset de variables.
		bits_counter	= 0;
		si				= 0;
		temp			= 0;
		temp_aux		= 0;
		ni_aux			= ni;
		
		//si "ni" es distinto de cero, entonces.
		if (ni != 0)
		{
			//bucle para obtener los primeros bits de "di".
			do
			{
				//obtención de bits remanentes.
				remaining_bits_amount = ni_aux - (bit_position + 1);

				//si "di", no est� totalmente en un byte, entonces.
				if (remaining_bits_amount > 0)
				{
					//obtención de los primeros bits de "di".
					temp_aux = _stream_com_data[i1] & ((1 << (ni_aux - remaining_bits_amount)) - 1);
					
					//seteo de los primeros bits de "di".
					this->joinBits(temp_aux, ni_aux - remaining_bits_amount, temp);
					
					//actualizaciones.
					bit_position			= 7;
					ni_aux					-= ni_aux - remaining_bits_amount;
					remaining_bits_amount	= 0;
					++i1;
				}

				//si "di", est� totalmente en un byte, entonces.
				else break;				

			} while (true);
			
			//lectura de código.
			temp = (temp << ni_aux) | ((_stream_com_data[i1] >> (bit_position - ni_aux + 1)) & ((1 << ni_aux) - 1));
			
			//seteo de "di".
			if (this->getBitsAmount(temp) == ni)	this->di_lec[di_index] = temp;
			else									this->di_lec[di_index] = temp - int(pow(2, ni)) + 1;
			
			//decremento de "bit_position".
			bit_position -= ni_aux;

			//si "bit_position" alcanza su mínimo valor, entonces.
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
			//obtención de "di".
			this->di_lec[di_index] = 0;
		}
		
		//incremento del índice de "di".
		++di_index;	

		//si sobrepasa el número de datos, entonces.
		if ((di_index + 1) > _samples_amount[data_type_index])
		{
			//obtención del primer dato.
			_stream_decom_data[data_type_index][0] = this->di_lec[0];
			
			//bucle para obtener datos de salida.
			for (int i2 = 1; i2 < _samples_amount[data_type_index]; i2++)
			{
				//obtención de datos.
				_stream_decom_data[data_type_index][i2] = _stream_decom_data[data_type_index][i2 - 1] + this->di_lec[i2];				
			}

			//incremento del índice del tipo de dato.
			++data_type_index;

			//si sobrepasa el número de datos, entonces salir del bucle.
			if (data_type_index > (_data_types_amount - 1)) break;

			//reset del índice de 'di'.
			di_index = 0;
			
			//limpieza de buffers.
			this->cleanBuffersLEC();
		}

	} while (true);

	//bucle para obtener la cantidad de bytes resultantes.
	for (int i1 = 0; i1 < _data_types_amount; i1++)
	{
		//cálculo de bytes obtenidos de la descompresión.
		decompress_size += int(_samples_amount[i1] * ceil(float(_bits_resolution_data[i1]) / 8.f));
	}	

	//retorno de la cantidad de bytes resultantes de la descompresión.
	return decompress_size;
}

//-------------------------------------------------------------------------------------
//método de descompresión LEC 3 para un solo tipo de datos.
int compressor_lec::decompressorLEC(unsigned char *_stream_com_data, int _samples_amount, int *_stream_decom_data)
{
	//variables auxiliares.
	int i1						= 0;
	int si						= 0;
	int ni						= 0;
	int ni_aux					= 0;
	int di_index				= 0;
	int	bit_position			= 7;
	int bits_counter			= 0;			
	int	remaining_bits_amount	= 0;
	int temp					= 0;
	int temp_aux				= 0;

	//bucle para descomprimir datos.  
	do
	{
		//bucle para obtener "ni".
		do
		{
			//obtención de "si".
			if ((_stream_com_data[i1] & (1 << bit_position)) == 0)	si = si << 1;
			else													si = (si << 1) | 1;
			
			//decremento de "bit_position".
			--bit_position;

			//si "bit_position" alcanza su mínimo valor, entonces.
			if (bit_position == -1)
			{
				//actualizaciones.
				bit_position = 7;
				++i1;
			}

			//incremento del contador de bits.
			++bits_counter;
			
			//obtención de "ni".
			ni = this->getNi(si, bits_counter);

			//si se ha obtenido "bi", entonces.
			if (ni != -1) break;

		} while (true);
		
		//reset de variables.
		bits_counter	= 0;
		si				= 0;
		temp			= 0;
		temp_aux		= 0;
		ni_aux			= ni;
		
		//si "ni" es distinto de cero, entonces.
		if (ni != 0)
		{
			//bucle para obtener los primeros bits de "di".
			do
			{
				//obtención de bits remanentes.
				remaining_bits_amount = ni_aux - (bit_position + 1);

				//si "di", no est� totalmente en un byte, entonces.
				if (remaining_bits_amount > 0)
				{
					//obtención de los primeros bits de "di".
					temp_aux = _stream_com_data[i1] & ((1 << (ni_aux - remaining_bits_amount)) - 1);

					//seteo de los primeros bits de "di".
					this->joinBits(temp_aux, ni_aux - remaining_bits_amount, temp);

					//actualizaciones.
					bit_position			= 7;
					ni_aux					-= ni_aux - remaining_bits_amount;
					remaining_bits_amount	= 0;
					++i1;
				}

				//si "di", est� totalmente en un byte, entonces.
				else break;				

			} while (true);

			//lectura de código.
			temp = (temp << ni_aux) | ((_stream_com_data[i1] >> (bit_position - ni_aux + 1)) & ((1 << ni_aux) - 1));
			
			//seteo de "di".
			if (this->getBitsAmount(temp) == ni)	this->di_lec[di_index] = temp;
			else									this->di_lec[di_index] = temp - int(pow(2, ni)) + 1;
			
			//decremento de "bit_position".
			bit_position -= ni_aux;

			//si "bit_position" alcanza su mínimo valor, entonces.
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
			//obtención de "di".
			this->di_lec[di_index] = 0;
		}
		
		//incremento del índice de "di".
		++di_index;	

		//si sobrepasa el número de datos, entonces salir.
		if ((di_index + 1) > _samples_amount) break;

	} while (true);

	//obtención del primer dato.
	_stream_decom_data[0] = this->di_lec[0];

	//bucle para obtener datos de salida.
	for (int i2 = 1; i2 < _samples_amount; i2++)
	{
		//obtención de datos.
		_stream_decom_data[i2] = _stream_decom_data[i2 - 1] + this->di_lec[i2];
	}

	//limpieza de buffers.
	this->cleanBuffersLEC();

	//retorno de la cantidad de bytes resultantes de la descompresión.
	return int(_samples_amount * ceil(float(this->bits_resolution_data) / 8.f));
}


//-------------------------------------------------------------------------------------
//método para inicializar tablas.
void compressor_lec::iniTableLEC()
{
	//incialización de valores de tabla.
	this->table_lec[0][0]	= 0;
	this->table_lec[1][0]	= 2;
	this->table_lec[2][0]	= 3;
	this->table_lec[3][0]	= 4;
	this->table_lec[4][0]	= 5;
	this->table_lec[5][0]	= 6;
	this->table_lec[6][0]	= 14;
	this->table_lec[7][0]	= 30;
	this->table_lec[8][0]	= 62;
	this->table_lec[9][0]	= 126;
	this->table_lec[10][0]	= 254;
	this->table_lec[11][0]	= 510;
	this->table_lec[12][0]	= 1022;
	this->table_lec[13][0]	= 2046;
	this->table_lec[14][0]	= 4094;

	//incialización de cantidad de bits por valores de tabla.
	this->table_lec[0][1]	= 2;
	this->table_lec[1][1]	= 3;
	this->table_lec[2][1]	= 3;
	this->table_lec[3][1]	= 3;
	this->table_lec[4][1]	= 3;
	this->table_lec[5][1]	= 3;
	this->table_lec[6][1]	= 4;
	this->table_lec[7][1]	= 5;
	this->table_lec[8][1]	= 6;
	this->table_lec[9][1]	= 7;
	this->table_lec[10][1]	= 8;
	this->table_lec[11][1]	= 9;
	this->table_lec[12][1]	= 10;
	this->table_lec[13][1]	= 11;
	this->table_lec[14][1]	= 12;
}

//-------------------------------------------------------------------------------------
//método para obtener "si" en tabla.
void compressor_lec::getSi(int _ni, int &_si, int &_busy_bits_si)
{
	//si '_ni' está debajo del patrón, entonces.
	if (_ni < 5)
	{
		//obtención de "si" según tabla.
		_si				= this->table_lec[_ni][0];
		_busy_bits_si	= this->table_lec[_ni][1];
	}	

	//si '_ni' está sobre del patrón, entonces.
	else
	{
		//obtención de "si" según fórmula.
		_si				= int(pow(2, _ni - 2)) - 2;
		_busy_bits_si	= _ni - 2;
	}
}

//-------------------------------------------------------------------------------------
//método para obtener "ni" en tabla.
int compressor_lec::getNi(int _si, int _busy_bits_si)
{
	//si '_si' está debajo del patrón, entonces.
	if (_si < 6)
	{
		//bucle para cotejar "_si" en tabla.
		for (int i1 = 0; i1 < 15; i1++)
		{
			//si existe en tabla, entonces.
			if (_si == table_lec[i1][0] && _busy_bits_si == table_lec[i1][1]) return i1;
		}
	}

	//si '_si' está sobre del patrón, entonces.  
	else
	{
		//variable auxiliar.
		int ni		= 5;
		int si_aux	= 0;

		//bucle para cotejar "_si" en tabla.
		while(true)
		{
			//obtención de 'si_aux'.
			si_aux = int(pow(2, ni - 2)) - 2;

			//si existe en tabla, entonces.
			if (_si == si_aux) return ni;

			//si no hay correspondencia, entonces salir.
			if (si_aux > _si) break;

			//incremento de 'ni'.
			++ni;
		}
	}

	//si no existe en tabla, entonces.
	return -1;
}

//-------------------------------------------------------------------------------------
//método para setear resolución en bits de los datos.
void compressor_lec::setBitsResolutionData(int _bits_resolution_data)
{
	//seteo de resolución en bits de los datos.
	this->bits_resolution_data = _bits_resolution_data;
}

//-------------------------------------------------------------------------------------
//método para limpiar buffers.
void compressor_lec::cleanBuffersLEC()
{
	//bucle para limpiar buffers.
	for (int i1 = 0; i1 < this->max_samples_amount; i1++)
	{
		//limpieza.
		this->di_lec[i1] = 0;
	}
}
