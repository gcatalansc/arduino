#pragma once

//inclusiones.
#include "compressor_sn.h"

//clase para compresor ALDC.
class compressor_aldc :	compressor_sn
{

//miembros privados.
private:

	//buffers.
	int				*di_aldc;																	//diferencias entre xi y xi_1 para ALDC;
	int				tables_aldc[3][15][2];														//tablas de c�digos Huffman para ALDC.
	unsigned char	*ci_a;																		//buffer auxiliar para ci_a.
	unsigned char	*ci_b;																		//buffer auxiliar para ci_b.
	unsigned char	*ci_c;																		//buffer auxiliar para ci_c.
	int				ci_a_index_available;														//�ndice de buffer "ci_a" disponible para realizar concatenaci�n.
	int				ci_b_index_available;														//�ndice de buffer "ci_b" disponible para realizar concatenaci�n.
	int				ci_c_index_available;														//�ndice de buffer "ci_c" disponible para realizar concatenaci�n.
	int				ci_a_available_bits;														//cantidad de bits disponibles para realizar concatenaci�n en buffer "ci_a".
	int				ci_b_available_bits;														//cantidad de bits disponibles para realizar concatenaci�n en buffer "ci_b".
	int				ci_c_available_bits;														//cantidad de bits disponibles para realizar concatenaci�n en buffer "ci_c".

//miembros p�blicos.
public:

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//m�todos para constructor y destructor.

	//-------------------------------------------------------------------------------------
	//m�todo constructor 1.
	compressor_aldc();

	//-------------------------------------------------------------------------------------
	//m�todo constructor 2.
	compressor_aldc(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size);

	//-------------------------------------------------------------------------------------
	//m�todo destructor.
	~compressor_aldc();

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//funciones para ALDC.

	//-------------------------------------------------------------------------------------
	//m�todo inicializaci�n
	void initialization(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size);

	//-------------------------------------------------------------------------------------
	//m�todo de compresi�n ALDC.
	int compressorALDC(int *_data_in, int _samples_amount, int _bits_resolution_data, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//-------------------------------------------------------------------------------------
	//m�todo de descompresi�n ALDC.
	int decompressorALDC(unsigned char *_stream_com_data, int _samples_amount, int *_data_out);

	//-------------------------------------------------------------------------------------
	//m�todo para codificar con tabla 2.
	void IITableALECEncoder(int *_di, int _samples_amount, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//-------------------------------------------------------------------------------------
	//m�todo para codificar con tabla 3.
	void IIITableALECEncoder(int *_di, int _samples_amount, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//-------------------------------------------------------------------------------------
	//m�todo para codificar "di".
	void encode(int _di, int _table, unsigned char *_ci, int &_busy_bits_ci, int &_ci_index_available, int &_ci_available_bits);

	//-------------------------------------------------------------------------------------
	//m�todo para inicializar tablas.
	void iniTablesALDC();

	//-------------------------------------------------------------------------------------
	//m�todo para obtener "DR".
	int getDR(int _F, int _samples_amount);

	//-------------------------------------------------------------------------------------
	//m�todo para obtener "hi" en tabla.
	void getHi(int _bi, int _table, int &_hi, int &_busy_bits_hi);

	//-------------------------------------------------------------------------------------
	//m�todo para obtener "bi" en tabla.
	int getBi(int _hi, int _busy_bits_hi, int _table);

	//-------------------------------------------------------------------------------------
	//m�todo para obtener "index".
	int getIndex(int _di, int _bi);

	//-------------------------------------------------------------------------------------
	//m�todo para limpiar buffers.
	void cleanBuffersALDC();
};

