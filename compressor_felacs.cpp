
//inclusiones.
#include "compressor_felacs.h"

//-------------------------------------------------------------------------------------
//m�todo constructor 1.
compressor_felacs::compressor_felacs()
{
	
}

//-------------------------------------------------------------------------------------
//m�todo constructor 2.
compressor_felacs::compressor_felacs(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size)
{
	//data settings.
	this->bits_resolution_data	= _bits_resolution_data;									//resoluci�n en bits de los datos.
	this->max_samples_amount	= _max_samples_amount;										//cantidad maxima de muestras a tomar.
	this->max_buffer_size		= _max_buffer_size;											//m�ximo tama�o del buffer de compresi�n de datos.

	//buffers.
	this->di_felacs				= new int[_max_samples_amount];								//diferencias entre xi y xi_1 para ALDC;
	this->delta					= new int[_max_samples_amount];								//deltas.

	//inicializaci�n.
	this->cleanBuffersFELACS();
}

//-------------------------------------------------------------------------------------
//m�todo destructor.
compressor_felacs::~compressor_felacs()
{

}

//-------------------------------------------------------------------------------------
//m�todo inicializaci�n.
void compressor_felacs::initialization(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size)
{
	//data settings.
	this->bits_resolution_data	= _bits_resolution_data;									//resoluci�n en bits de los datos.
	this->max_samples_amount	= _max_samples_amount;										//cantidad maxima de muestras a tomar.
	this->max_buffer_size		= _max_buffer_size;											//m�ximo tama�o del buffer de compresi�n de datos.

	//buffers.
	this->di_felacs				= new int[max_samples_amount];								//diferencias entre xi y xi_1 para ALDC;
	this->delta					= new int[max_samples_amount];								//deltas.

	//inicializaci�n.
	this->cleanBuffersFELACS();
}

//-------------------------------------------------------------------------------------
//m�todo de compresi�n FELACS.
int compressor_felacs::compressorFELACS(int *_data_in, int _samples_amount, int _bits_resolution_data, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits)
{
	//variables auxiliares.	
	int j				= 0;
	int	D				= 0;
	int	k				= 0;
	int code			= 0;
	int busy_bits_code	= 0;

	//seteo del primer elemento de "di" y de "this->delta".
	this->di_felacs[0]	= _data_in[0];
	this->delta[0]		= this->di_felacs[0];
	
	//bucle para obtenci�n de los "di".
	for (int i1 = 1; i1 < _samples_amount; i1++)
	{
		//obtenci�n de los "di".
		this->di_felacs[i1] = _data_in[i1] - _data_in[i1 - 1];
	}

	//bucle para obtenci�n de los "deltas".
	for (int i1 = 1; i1 < _samples_amount; i1++)
	{
		//obtenci�n de los "this->delta".
		this->delta[i1] = this->getDelta(this->di_felacs[i1], _data_in[i1 - 1], _bits_resolution_data);
	}
	
	//c�lculo de "j".
	j = _samples_amount - 1;
	
	//bucle para c�lcular "D".
	for (int i1 = 1; i1 < _samples_amount; i1++)
	{
		//c�lculo de "D".
		D += this->delta[i1];
	}
	
	//obtenci�n de "k".
	k			= getK(D, j);
	
	//seteo de "k" y de "this->delta[0]" en buffer "_stream_com_data".
	this->joinBits(k, 3, _stream_com_data, _stream_index_available, _stream_available_bits);
	this->joinBits(this->delta[0], this->bits_resolution_data, _stream_com_data, _stream_index_available, _stream_available_bits);
	
	//bucle para obtener c�digos "deltas" y setearlos en buffer "_stream_com_data".
	for (int i1 = 1; i1 < _samples_amount; i1++)
	{
		//obtenci�n de c�digos Golomb-RiceCode de los "deltas".
		this->getGolombRiceCode(k, this->delta[i1], code, busy_bits_code);
		
		//seteo de c�digo en buffer "stream_com_data".
		this->joinBits(code, busy_bits_code, _stream_com_data, _stream_index_available, _stream_available_bits);

		//reset de variables auxiliares.
		code			= 0;
		busy_bits_code	= 0;
	}

	//limpieza de "deltas".
	this->cleanDeltas();

	//retorno de la cantidad de bytes resultantes de la compresi�n.
	if (_stream_available_bits == 8)	return _stream_index_available;
	else								return _stream_index_available + 1;
}

//-------------------------------------------------------------------------------------
//m�todo de descompresi�n FELACS.
int compressor_felacs::decompressorFELACS(unsigned char *_stream_com_data, int _samples_amount, int *_data_out)
{
	//variables auxiliares.
	int				k							= 0;
	int				k_aux						= 0;
	int				delta_counter				= 0;
	int				zeros_counter				= 0;
	int				prefix						= 0;
	int				suffix						= 0;
	int				bit_position				= 0;	
	int				remaining_bits_amount		= 0;
	int				index_amount				= int(floor((this->bits_resolution_data - 5)/8.0f));
	int				temp						= 0;
	int				temp_aux					= 0;
	int				prefix_bits_amount			= 0;
	int				i1							= 0;

	//obtenci�n de "k".
	k			= _stream_com_data[0] >> 5;
	
	//obtenci�n de los primeros 5 bits de "this->delta[0]".
	temp		= _stream_com_data[0] & ((1 << 5) - 1);
	
	//seteo primeros 5 bits de "this->delta[0]".
	this->joinBits(temp, 5, this->delta[0]);

	//bucle para setear los bytes restantes de "this->delta[0]".
	for (i1 = 1; i1 < (index_amount + 1); i1++)
	{
		//seteo de bytes restantes de "this->delta[0]".
		this->joinBits(_stream_com_data[i1], 8, this->delta[0]);
	}
	
	//bits remanentes de "this->delta[0]".
	remaining_bits_amount = this->bits_resolution_data - (5 + 8 * index_amount);
	
	//si existen bits restantes de "this->delta[0]", entonces.
	if (remaining_bits_amount > 0)
	{
		//seteo de los bits restantes de "this->delta[0]".
		temp	= _stream_com_data[i1] >> (8 - remaining_bits_amount);
		this->joinBits(temp, remaining_bits_amount, this->delta[0]);
	}
	
	//actualizaci�n de variables auxiliares.
	bit_position = 7 - remaining_bits_amount;	
		
	//bucle para obtener los "deltas" restantes.
	do
	{	
		//si hay un cero, entonces.
		if ((_stream_com_data[i1] & (1 << bit_position)) == 0)
		{			
			//reset de variables.
			prefix = 0;
			suffix = 0;
			
			//incremento del contador de ceros.
			++zeros_counter;

			//decremento de "bit_position".
			--bit_position;

			//si "bit_position" alcanza su m�nimo valor, entonces.
			if (bit_position == -1)
			{
				//actualizaciones.
				bit_position = 7;
				++i1;
			}
		}

		//si no hay un cero, entonces.
		else
		{
			//seteo de prefijo del "this->delta".
			prefix				= zeros_counter;		
			prefix_bits_amount	= this->getBitsAmount(prefix);
						
			//reset y actualizaci�n de variables.			
			++delta_counter;
			zeros_counter = 0;

			//decremento de "bit_position".
			bit_position -= 1;
						
			//si el "prefix" es mayor que cero, entonces.
			if (prefix > 0)
			{
				//concatenaci�n del prefijo para obtener "this->delta".
				this->joinBits(prefix, prefix_bits_amount, this->delta[delta_counter]);
			}			
			
			//reset de variables.
			temp			= 0;
			temp_aux		= 0;
			k_aux			= k;
			
			//bucle para obtener los primeros bits de "this->delta".
			do
			{
				//obtenci�n de bits remanentes.
				remaining_bits_amount = k_aux - (bit_position + 1);
								
				//si "this->delta", no est� totalmente en un byte, entonces.
				if (remaining_bits_amount > 0)
				{
					//obtenci�n de los primeros bits de "this->delta".
					temp_aux = _stream_com_data[i1] & ((1 << (k_aux - remaining_bits_amount)) - 1);

					//seteo de los primeros bits de "di".
					this->joinBits(temp_aux, k_aux - remaining_bits_amount, temp);

					//actualizaciones.
					bit_position			= 7;
					k_aux					-= k_aux - remaining_bits_amount;
					remaining_bits_amount	= 0;
					++i1;
				}

				//si "this->delta", est� totalmente en un byte, entonces.
				else break;				

			} while (true);

			//lectura de c�digo.
			suffix = (temp << k_aux) | ((_stream_com_data[i1] >> (bit_position - k_aux + 1)) & ((1 << k_aux) - 1));
			
			//concatenaci�n del sufijo para obtener "this->delta".
			this->joinBits(suffix, k, this->delta[delta_counter]);
			
			//decremento de "bit_position".
			bit_position			-= k_aux;

			//si "bit_position" alcanza su m�nimo valor, entonces.
			if (bit_position == -1)
			{
				//actualizaciones.
				bit_position = 7;
				++i1;
			}
		}

		//si es el �ltimo "this->delta" a obtener, entonces.
		if ((delta_counter + 1) == _samples_amount) break;

	} while (true);

	//obtenci�n de "di[0]".
	this->di_felacs[0]	= this->delta[0];

	//obtenci�n de "data_out[0]".
	_data_out[0]		= this->di_felacs[0];

	//bucle para obtener los "di".
	for (i1= 1; i1 < _samples_amount; i1++)
	{
		//obtenci�n de "di[i1]".
		this->di_felacs[i1]	= this->getDiFELACS(this->delta[i1], _data_out[i1 - 1], this->bits_resolution_data);
		
		//obtenci�n de "data_out[i1]".
		_data_out[i1]	= _data_out[i1 - 1] + this->di_felacs[i1];
	}

	//limpieza de buffers.
	this->cleanBuffersFELACS();

	//retorno de la cantidad de bytes resultantes de la descompresi�n.
	return (_samples_amount * ceil(float(this->bits_resolution_data) / 8.f));
}

//-------------------------------------------------------------------------------------
//m�todo para obtener "di".
int compressor_felacs::getDiFELACS(int _delta, int _xi_1, int _bits_resolution_data)
{
	//variables.
	int theta	= this->minor(_xi_1, pow(2, _bits_resolution_data) - 1 - _xi_1);
	int di		= 0;
	int rest	= 0;
	//obtenci�n del resto.
	rest = _delta % 2;

	//si no existe resto, entonces.
	if (rest == 0)
	{
		//inicializaci�n de "di".
		di = _delta / 2;

		//obtenci�n de "di".
		return di;
	}

	//si no existe resto, entonces.
	else
	{
		//inicializaci�n de "di".
		di = -((_delta + 1) / 2);

		//obtenci�n de "di".
		if (di >= -theta && di < 0)	return di;

		//obtenci�n de "di".
		return (-(_delta - theta));
	}
}

//-------------------------------------------------------------------------------------
//m�todo para obtener "this->delta".
int compressor_felacs::getDelta(int _di, int _xi_1, int _bits_resolution_data)
{
	//variables.
	int theta = this->minor(_xi_1, pow(2, _bits_resolution_data) - 1 - _xi_1);

	//obtenci�n de "this->delta".
	if (_di >= 0 && _di <= theta)	return (2 * _di);
	if (_di >= -theta && _di < 0)	return (2 * abs(_di) - 1);
	else							return (theta + abs(_di));
}

//-------------------------------------------------------------------------------------
//m�todo para obtener "k".
int compressor_felacs::getK(int _D, int _j)
{
	//variable auxiliar.
	int i = 0;
	int k = 0;

	//bucle para obtener "k".
	do
	{
		//si se cumple, entonces setear "k".
		if (_D <= pow(2, ++i)*_j)	return k;

		//incremento de "k".
		++k;

	} while (true);
}

//-------------------------------------------------------------------------------------
//m�todo para obtener c�digo Golomb-Rice.
void compressor_felacs::getGolombRiceCode(int _k, int _delta, int &_output_code, int &_busy_bits_code)
{
	//variables.
	int	zeros_amount				= int(_delta/(pow(2, _k)));
	int	suffix						= _delta & ((1 << _k) - 1);
	int	output_code_available_bits	= sizeof(int)*8;
	int	l							= 0;

	//si "zeros_amount" es mayor que cero, entonces.
	if (zeros_amount > 0)
	{
		//uni�n de los bits.
		this->joinBits(0, zeros_amount, _output_code, output_code_available_bits);
		this->joinBits(1, 1, _output_code, output_code_available_bits);
		this->joinBits(suffix, _k, _output_code, output_code_available_bits);

		//seteo de "l".
		l = zeros_amount + 1 + _k;
	}

	//si "zeros_amount" es igual a  cero, entonces.
	else
	{
		//uni�n de los bits.
		this->joinBits(1, 1, _output_code, output_code_available_bits);
		this->joinBits(suffix, _k, _output_code, output_code_available_bits);

		//seteo de "l".
		l = _k + 1;
	}	
	
	//seteo de la cantidad de bits ocupados.
	_busy_bits_code = l;
}

//-------------------------------------------------------------------------------------
//m�todo para limpiar "deltas".
void compressor_felacs::cleanDeltas(void)
{
	//bucle para limpiar "deltas".
	for (int i1 = 0; i1 < this->max_samples_amount; i1++)
	{
		//limpieza.
		this->delta[i1] = 0;
	}
}

//-------------------------------------------------------------------------------------
//m�todo para limpiar buffers.
void compressor_felacs::cleanBuffersFELACS()
{
	//bucle para limpiar "di" y "this->delta".
	for (int i1 = 0; i1 < this->max_samples_amount; i1++)
	{
		//limpieza.
		this->di_felacs[i1]	= 0;
		this->delta[i1]		= 0;
	}
}

