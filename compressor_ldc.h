#pragma once

//inclusiones.
#include "compressor_sn.h"

//clase para compresor LDC.
class compressor_ldc : public compressor_sn
{
//miembros privados.
private:

	//estructura para generación de árbol.
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
	int					**table_ldc;															//tabla de código Huffman para LDC.
	int					**fdi;																	//frecuencias de "di".
	int					length_dataset;															//largo del vector dataset.
	int					length_fdi;																//cantidad de "di".
	treeHuffmanCode		tree_huffman_code;														//árbol para codificación huffman.

//miembros públicos.
public:

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//métodos para constructor y destructor.

	//-------------------------------------------------------------------------------------
	//método constructor 1.
	compressor_ldc();

	//-------------------------------------------------------------------------------------
	//método constructor 2.
	compressor_ldc(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size, int *_dataset, int _length_dataset);

	//-------------------------------------------------------------------------------------
	//método destructor.
	~compressor_ldc();

	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------
	//métodos para LDC.

	//-------------------------------------------------------------------------------------
	//método de inicialización.
	void initialization(int _bits_resolution_data, int _max_samples_amount, int _max_buffer_size, int *_dataset, int _length_dataset);

	//-------------------------------------------------------------------------------------
	//método de compresión LDC.
	int compressorLDC(int *_data_in, int _samples_amount, int _bits_resolution_data, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);

	//-------------------------------------------------------------------------------------
	//método de descompresión LDC.
	int decompressorLDC(unsigned char *_stream_com_data, int _samples_amount, int *_data_out);

	//-------------------------------------------------------------------------------------
	//método para obtener la cantidad de "di" diferentes.
	int getDiCounter();

	//-------------------------------------------------------------------------------------
	//método para setear los "di_set".
	void setDi(int *_dataset_in);

	//-------------------------------------------------------------------------------------
	//método para obtener la frecuencia ordenadas de menor a mayor de cada "di_set".
	void setFDI();

	//-------------------------------------------------------------------------------------
	//método para ordenadar las frecuencias de menor a mayor de cada "di_set".
	void orderFDI(void);

	//-------------------------------------------------------------------------------------
	//método para comprobar si existe "di_set" en buffer "fdi".
	bool checkDiOnFdi(int _di_set);

	//-------------------------------------------------------------------------------------
	//método para inicializar tabla.
	void iniTableLDC(treeHuffmanCode &_tree_huffman_code, int *_dataset);

	//-------------------------------------------------------------------------------------
	//método para crear tabla de códigos huffman.
	void createTableLDC(treeHuffmanCode &_node, int &_index_table);

	//-------------------------------------------------------------------------------------
	//método para obtener árbol de código huffman tipo 1.
	void getHuffmanTree(treeHuffmanCode &_tree_huffman_code);

	//-------------------------------------------------------------------------------------
	//método para obtener "huffman code" en tabla.
	void getHuffmanCode(int _di, int &_code, int &_busy_bits_code);

	//-------------------------------------------------------------------------------------
	//método para obtener "di" en tabla.
	int getDiLDC(int _code, int _busy_bits_code);
	
	//-------------------------------------------------------------------------------------
	//método para obtener caontenido den tabla.
	int getTable(int _row, int _columm);
	
	//-------------------------------------------------------------------------------------
	//método para obtener largo e fdi.
	int getLength_fdi();
	
	//-------------------------------------------------------------------------------------
	//método para obtener el máximo valor de una columna de la tabla.
	int maxTable(int _co);

	//-------------------------------------------------------------------------------------
	//método para limpiar buffers.
	void cleanBuffersLDC();

};

