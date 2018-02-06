
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//inclusiones.
#include "compressor_sn.h"

//-------------------------------------------------------------------------------------
//método constructor.
compressor_sn::compressor_sn()
{

}

//-------------------------------------------------------------------------------------
//método destructor.
compressor_sn::~compressor_sn()
{

}

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//métodos comunes.

//---------------------------------------------------------------------------------
//método para concatenar bits de una arreglo en otro arreglo.
bool compressor_sn::joinBits(unsigned char *_input_stream, int _busy_bits_input_stream, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits)
{
	//--------------------------------------------------------------------------------- 
	//variables auxiliares.		
	int				input_stream_available_bits = 0;
	unsigned char	temp						= 0;
	unsigned char	minor_or_equal				= 0;
	unsigned char	input_stream_index			= 0;
	unsigned char	bits_counter				= 0;

	//si el buffer no puede contener a "_input_stream".
	if (_busy_bits_input_stream > ((this->max_buffer_size - _stream_index_available)*8 - (8 - _stream_available_bits))) return false;

	//si el buffer puede contener a "_input_stream".
	else
	{
		//inicialización de los bits disponibles para concatenar en "_input_stream".
		input_stream_available_bits = _busy_bits_input_stream;

		//bucle para concatenar bits.  
		do
		{
			//obtención del menor número.
			minor_or_equal	= minor(_stream_available_bits, input_stream_available_bits);

			//actualización del contador de bits.
			bits_counter	+= minor_or_equal;

			//obtención de los "minor_or_equal" bits de "_input_stream" a concatenar. 
			if (input_stream_available_bits <= 8)	temp = ((1 << minor_or_equal) - 1) & (_input_stream[input_stream_index] >> (input_stream_available_bits - minor_or_equal));
			else									temp = ((1 << minor_or_equal) - 1) & (_input_stream[input_stream_index] >> (8 - minor_or_equal));

			//concatenación de bits de "_input_stream" a "_stream_com_data".    
			_stream_com_data[_stream_index_available] = _stream_com_data[_stream_index_available] | (temp << (_stream_available_bits - minor_or_equal));
			
			//actualización de los bits disponibles en "_stream_com_data".
			_stream_available_bits		-= minor_or_equal;

			//actualización de la cantidad de bits disponibles en "_input_stream".
			input_stream_available_bits -= minor_or_equal;

			//si se ha llenado la posición "_stream_index_available".
			if (_stream_available_bits == 0)
			{
				//incremento del índice de buffer "_stream_com_data" a utilizar.
				++_stream_index_available;

				//reset de los bits disponibles en "_stream_com_data".
				_stream_available_bits = 8;
			}

			//si se han usado todos los bits de un byte de "_input_stream", entonces.
			if (bits_counter = 8)
			{
				//incremento del índice de buffer "input_stream_index" utilizado.
				++input_stream_index;

				//reset de los bits disponibles en "input_stream_index".
				bits_counter = 0;
			}

		//si hay bits disponibles a concatenar, entonces continuar.
		} while (input_stream_available_bits > 0);
	}
	
	//retorno positivo.
	return true;
}

//-------------------------------------------------------------------------------------
//método para concatenar bits en un arreglo.
void compressor_sn::joinBits(int _code, int _busy_bits_code, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits)
{
	//--------------------------------------------------------------------------------- 
	//variables auxiliares.		
	unsigned char	code_available_bits = 0;
	unsigned char	temp				= 0;
	unsigned char	minor_or_equal		= 0;

	//si el buffer no puede contener a "_code".
	if (_busy_bits_code > ((this->max_buffer_size - _stream_index_available)*8 - (8 - _stream_available_bits)))
	{
		//salida.
		return;
	}

	//si el buffer puede contener a "_code".
	else
	{
		//inicialización de los bits disponibles para concatenar en "_code".
		code_available_bits = _busy_bits_code;

		//bucle para concatenar bits.  
		do
		{
			//obtención del menor número.
			minor_or_equal	= minor(_stream_available_bits, code_available_bits);

			//obtención de los bits de "_code" a concatenar. 
			temp			= ((1 << minor_or_equal) - 1) & (_code >> (code_available_bits - minor_or_equal));

			//concatenación de bits de "_code" a "_stream_com_data".    
			_stream_com_data[_stream_index_available] = _stream_com_data[_stream_index_available] | (temp << (_stream_available_bits - minor_or_equal));

			//actualización de los bits disponibles en "_stream_com_data".
			_stream_available_bits	-= minor_or_equal;

			//actualización de la cantidad de bits disponibles en "_code".
			code_available_bits		-= minor_or_equal;

			//si se ha llenado la posición "_stream_index_available".
			if (_stream_available_bits == 0)
			{
				//incremento del índice de buffer "_stream_com_data" a utilizar.
				++_stream_index_available;

				//reset de los bits disponibles en "_stream_com_data".
				_stream_available_bits = 8;
			}

		//si hay bits disponibles a concatenar, entonces continuar.
		} while (code_available_bits > 0);
	}
}

//-------------------------------------------------------------------------------------
//método para concatenar bits en una variable.
void compressor_sn::joinBits(int _code, int _busy_bits_code, int &_stream_com_data, int &_stream_available_bits)
{
	//concatenaci�n de bits de "_code" a "_stream_com_data". 
	_stream_com_data = (_stream_com_data << _busy_bits_code) | (_code & ((1 << _busy_bits_code) - 1));

	//actualizaci�n de los bits disponibles en "_stream_com_data".
	_stream_available_bits -= _busy_bits_code;
}

//-------------------------------------------------------------------------------------
//método para concatenar bits en una variable.
void compressor_sn::joinBits(int _code, int _busy_bits_code, int &_stream_com_data)
{
	//concatenaci�n de bits de "_code" a "_stream_com_data". 
	_stream_com_data = (_stream_com_data << _busy_bits_code) | (_code & ((1 << _busy_bits_code) - 1));
}

//-------------------------------------------------------------------------------------
//método para calcular log2 de "di".
int	compressor_sn::getLog2Di(int _di)
{
	//auxiliar data.
	int	aux_di	= _di;
	int	ni		= 0;

	//bucle para calcular log2 de di.
	while (aux_di > 0)
	{
		//calculo de log2 de di.
		aux_di = aux_di / 2;
		++ni;
	}

	//retorno de ni.
	return ni;
}

//-------------------------------------------------------------------------------------
//método para obtener la cantidad de bites utilizados en un registro entero.
int compressor_sn::getBitsAmount(int _code)
{
	//variables auxiliares.
	int int_bytes_amount = sizeof(_code);

	//si el valor es cero, entonces.
	if (_code == 0)	return 0;

	//---------------------------------------------------------------------------------
	//bucle para obtener cantidad bits utilizados.
	for (int i1 = (int_bytes_amount * 8 - 1); i1 >= 0; i1--)
	{
		//si existe un bit en 1, entonces.
		if ((_code & (1 << i1)) != 0)	return (i1 + 1);
	}

	//retorno por defecto.
	return -1;
}

//-------------------------------------------------------------------------------------
//método que devuelve el valor menor entre "_a" y  "_b".
int compressor_sn::minor(int _a, int _b)
{
	//retorno del valor menor o igual.
	if (_a <= _b)	return _a;
	return _b;
}
