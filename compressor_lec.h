//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//inicio compilaci�n condicional.
#ifndef __COMPRESS_LEC__
#define __COMPRESS_LEC__

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//inclusiones.
#include "compressor_sn.h"

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//clase para compresor LEC.
class compressor_lec : compressor_sn
{
	//miembros privados.
private:

	//---------------------------------------------------------------------------------
	//buffers.
	int	*di_lec;																			//diferencias entre xi y xi_1 para LEC.
	int	table_lec[15][2];																	//tabla para diccionario LEC.

//miembros p�blicos.
public:

	//---------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------
	//m�todos para constructor y destructor.

	//---------------------------------------------------------------------------------
	//m�todos constructor.
	compressor_lec();
	compressor_lec(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size);
	compressor_lec(int _max_samples_amount, int _max_buffer_size);

	//---------------------------------------------------------------------------------
	//m�todo destructor.
	~compressor_lec();

	//---------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------
	//m�todos para LEC.

	//---------------------------------------------------------------------------------
	//m�todos de inicializaci�n.
	void initialization(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size);
	void initialization(int _max_samples_amount, int _max_buffer_size);

	//---------------------------------------------------------------------------------
	//m�todos de compresi�n LEC. 
	int compressorLEC(int *_data_in, int _samples_amount, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);
	int compressorLEC(int _data_in_p, int _data_in, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//---------------------------------------------------------------------------------
	//m�todos de descompresi�n LEC.
	int decompressorLEC(unsigned char *_stream_com_data, int *_data_in_p, int *_remaining_data_types, int _remaining_data_type_amount, int _data_types_amount, int _base_samples_amount, int *_samples_amount, int *_bits_resolution_data, int **_stream_decom_data);
	int decompressorLEC(unsigned char *_stream_com_data, int _data_types_amount, int *_samples_amount, int *_bits_resolution_data, int **_stream_decom_data);
	int decompressorLEC(unsigned char *_stream_com_data, int _samples_amount, int *_stream_decom_data);

	//---------------------------------------------------------------------------------
	//m�todo para inicializar tablas.
	void iniTableLEC();

	//---------------------------------------------------------------------------------
	//m�todo para obtener "si" en tabla.
	void getSi(int _ni, int &_si, int &_busy_bits_si);

	//---------------------------------------------------------------------------------
	//m�todo para obtener "ni" en tabla.
	int getNi(int _si, int _busy_bits_si);

	//---------------------------------------------------------------------------------
	//m�todo para setear resoluci�n en bits de los datos.
	void setBitsResolutionData(int _bits_resolution_data);

	//---------------------------------------------------------------------------------
	//m�todo para limpiar buffers.
	void cleanBuffersLEC();
};

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//fin compilaci�n condicional.
#endif

