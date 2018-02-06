#pragma once

//inclusiones.
#include "compressor_sn.h"

//clase para compresor LDC.
class compressor_ldc : public compressor_sn
{
//miembros privados.
private:

	//estructura para generaci�n de �rbol.
	struct treeHuffmanCode
	{
		//variables
		int di		= 0;			//diferencia.
		int sum		= 0;			//suma acumulativa de frecuencias.

		//punteros.
		treeHuffmanCode *left_tree	= 0;
		treeHuffmanCode *right_tree	= 0;
		treeHuffmanCode *next_tree	= 0;
	};

	//buffers.
	int					*di_ldc;																//diferencias entre xi y xi_1 para LDC;
	int					*diset_ldc;																//diferencias entre xi y xi_1 para set de datos del algoritmo LDC.
	int					**table_ldc;															//tabla de c�digo Huffman para LDC.
	int					**fdi;																	//frecuencias de "di".
	int					length_dataset;															//largo del vector dataset.
	int					length_fdi;																//cantidad de "di".
	treeHuffmanCode		tree_huffman_code;														//�rbol para codificaci�n huffman.

//miembros p�blicos.
public:

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//m�todos para constructor y destructor.

	//-------------------------------------------------------------------------------------
	//m�todo constructor 1.
	compressor_ldc();

	//-------------------------------------------------------------------------------------
	//m�todo constructor 2.
	compressor_ldc(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size, int *_dataset, int _length_dataset);

	//-------------------------------------------------------------------------------------
	//m�todo destructor.
	~compressor_ldc();

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//m�todos para LDC.

	//-------------------------------------------------------------------------------------
	//m�todo de inicializaci�n.
	void initialization(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size, int *_dataset, int _length_dataset);

	//-------------------------------------------------------------------------------------
	//m�todo de compresi�n LDC.
	int compressorLDC(int *_data_in, int _samples_amount, int _bits_resolution_data, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//-------------------------------------------------------------------------------------
	//m�todo de descompresi�n LDC.
	int decompressorLDC(unsigned char *_stream_com_data, int _samples_amount, int *_data_out);

	//-------------------------------------------------------------------------------------
	//m�todo para obtener la cantidad de "di" diferentes.
	int getDiCounter();

	//-------------------------------------------------------------------------------------
	//m�todo para setear los "di_set".
	void setDi(int *_dataset_in);

	//-------------------------------------------------------------------------------------
	//m�todo para obtener la frecuencia ordenadas de menor a mayor de cada "di_set".
	void setFDI();

	//-------------------------------------------------------------------------------------
	//m�todo para ordenadar las frecuencias de menor a mayor de cada "di_set".
	void orderFDI(void);

	//-------------------------------------------------------------------------------------
	//m�todo para comprobar si existe "di_set" en buffer "fdi".
	bool checkDiOnFdi(int _di_set);

	//-------------------------------------------------------------------------------------
	//m�todo para inicializar tabla.
	void iniTableLDC(treeHuffmanCode &_tree_huffman_code, int *_dataset);

	//-------------------------------------------------------------------------------------
	//m�todo para crear tabla de c�digos huffman.
	void createTableLDC(treeHuffmanCode &_node, int &_index_table);

	//-------------------------------------------------------------------------------------
	//m�todo para obtener �rbol de c�digo huffman tipo 1.
	void getHuffmanTree(treeHuffmanCode &_tree_huffman_code);

	//-------------------------------------------------------------------------------------
	//m�todo para obtener "huffman code" en tabla.
	void getHuffmanCode(int _di, int &_code, int &_busy_bits_code);

	//-------------------------------------------------------------------------------------
	//m�todo para obtener "di" en tabla.
	int getDiLDC(int _code, int _busy_bits_code);
	
	//-------------------------------------------------------------------------------------
	//m�todo para obtener caontenido den tabla.
	int getTable(int _row, int _columm);
	
	//-------------------------------------------------------------------------------------
	//m�todo para obtener largo e fdi.
	int getLength_fdi();
	
	//-------------------------------------------------------------------------------------
	//m�todo para obtener el m�ximo valor de una columna de la tabla.
	int maxTable(int _co);

	//-------------------------------------------------------------------------------------
	//m�todo para limpiar buffers.
	void cleanBuffersLDC();

};

