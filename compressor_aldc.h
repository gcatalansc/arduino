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
	int				tables_aldc[3][15][2];														//tablas de códigos Huffman para ALDC.
	unsigned char	*ci_a;																		//buffer auxiliar para ci_a.
	unsigned char	*ci_b;																		//buffer auxiliar para ci_b.
	unsigned char	*ci_c;																		//buffer auxiliar para ci_c.
	int				ci_a_index_available;														//índice de buffer "ci_a" disponible para realizar concatenación.
	int				ci_b_index_available;														//índice de buffer "ci_b" disponible para realizar concatenación.
	int				ci_c_index_available;														//índice de buffer "ci_c" disponible para realizar concatenación.
	int				ci_a_available_bits;														//cantidad de bits disponibles para realizar concatenación en buffer "ci_a".
	int				ci_b_available_bits;														//cantidad de bits disponibles para realizar concatenación en buffer "ci_b".
	int				ci_c_available_bits;														//cantidad de bits disponibles para realizar concatenación en buffer "ci_c".

//miembros públicos.
public:

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//métodos para constructor y destructor.

	//-------------------------------------------------------------------------------------
	//método constructor 1.
	compressor_aldc();

	//-------------------------------------------------------------------------------------
	//método constructor 2.
	compressor_aldc(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size);

	//-------------------------------------------------------------------------------------
	//método destructor.
	~compressor_aldc();

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//funciones para ALDC.

	//-------------------------------------------------------------------------------------
	//método inicialización
	void initialization(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size);

	//-------------------------------------------------------------------------------------
	//método de compresión ALDC.
	int compressorALDC(int *_data_in, int _samples_amount, int _bits_resolution_data, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//-------------------------------------------------------------------------------------
	//método de descompresión ALDC.
	int decompressorALDC(unsigned char *_stream_com_data, int _samples_amount, int *_data_out);

	//-------------------------------------------------------------------------------------
	//método para codificar con tabla 2.
	void IITableALECEncoder(int *_di, int _samples_amount, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//-------------------------------------------------------------------------------------
	//método para codificar con tabla 3.
	void IIITableALECEncoder(int *_di, int _samples_amount, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//-------------------------------------------------------------------------------------
	//método para codificar "di".
	void encode(int _di, int _table, unsigned char *_ci, int &_busy_bits_ci, int &_ci_index_available, int &_ci_available_bits);

	//-------------------------------------------------------------------------------------
	//método para inicializar tablas.
	void iniTablesALDC();

	//-------------------------------------------------------------------------------------
	//método para obtener "DR".
	int getDR(int _F, int _samples_amount);

	//-------------------------------------------------------------------------------------
	//método para obtener "hi" en tabla.
	void getHi(int _bi, int _table, int &_hi, int &_busy_bits_hi);

	//-------------------------------------------------------------------------------------
	//método para obtener "bi" en tabla.
	int getBi(int _hi, int _busy_bits_hi, int _table);

	//-------------------------------------------------------------------------------------
	//método para obtener "index".
	int getIndex(int _di, int _bi);

	//-------------------------------------------------------------------------------------
	//método para limpiar buffers.
	void cleanBuffersALDC();
};

