//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//inicio compilación condicional.
#ifndef __COMPRESS_SN__
#define __COMPRESS_SN__

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//inclusiones.
#include <cmath>

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//nombre de espacios.
using namespace std;

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//clase para compresor para redes de sensores.
class compressor_sn
{

//miembros privados.
protected:

	//---------------------------------------------------------------------------------
	//data settings.
	int	bits_resolution_data;																//resolución en bits de los datos.
	int	max_samples_amount;																	//cantidad máxima de muestras a tomar.
	int max_buffer_size;																	//máximo tama�o del buffer de compresión de datos.

//miembros públicos.
public:	

	//---------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------
	//métodos para constructor y destructor.

	//---------------------------------------------------------------------------------
	//método constructor.
	compressor_sn();

	//---------------------------------------------------------------------------------
	//método destructor.
	~compressor_sn();

	//---------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------
	//métodos comunes.
	
	//---------------------------------------------------------------------------------
	//método para concatenar bits de una arreglo en otro arreglo.
	bool joinBits(unsigned char *_input_stream, int _busy_bits_input_stream, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//---------------------------------------------------------------------------------
	//método para concatenar bits en un arreglo.
	void joinBits(int _code, int _busy_bits_code, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//---------------------------------------------------------------------------------
	//método para concatenar bits en una variable.
	void joinBits(int _code, int _busy_bits_code, int &_stream_com_data, int &_stream_available_bits);

	//---------------------------------------------------------------------------------
	//método para concatenar bits en una variable.
	void joinBits(int _code, int _busy_bits_code, int &_stream_com_data);

	//---------------------------------------------------------------------------------
	//método para calcular log2 de "di".
	int	getLog2Di(int _di);

	//---------------------------------------------------------------------------------
	//método para obtener la cantidad de bites utilizados en un registro entero.
	int getBitsAmount(int _code);	

	//---------------------------------------------------------------------------------
	//método que devuelve el valor menor entre "_a" y  "_b".
	int minor(int _a, int _b);
};

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//fin compilación condicional.
#endif

