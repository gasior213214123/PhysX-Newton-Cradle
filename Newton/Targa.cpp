/*
 *	Uniwersalne procedury ³aduj¹ce pliki typu TARGA
 *	z uwzglêdnieniem kana³u ALFA.
 *	Na podstawie tutorial'u NEHE nr 25.
 */

#include "stdafx.h"

#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <gl/gl.h>
#include <gl/glu.h>

#include "Targa.h"

///////////////////////////////////////////////////////////////////////////////
// Procedura ³aduje plik o podanej nazwie
// wype³nia podan¹ strukturê TARGAINFO
// oraz zwraca adres bufora z pikselami (jeœli nie by³o b³êdu)
// Jeœli by³ b³¹d - zwraca NULL
GLubyte *LoadTGAImage(char *filename, TARGAINFO *info)
{
	GLubyte	TGAHeader[12]={0,0,2,0,0,0,0,0,0,0,0,0};	// Nag³ówek TGA bez kompresji
	GLubyte	TGACompare[12];			// Tu siê za³aduje dane z pliku
	GLubyte	Header[6];			// Pierwsze potrzebne 6 bajtów z pliku
	GLubyte *Bits = NULL;	// WskaŸnik na bufor z danymi pikseli

	FILE *plik;
	fopen_s(&plik, filename, "rb");	// Próba otwarcia do odczytu
	if(plik)
	{
		fread(TGACompare, 1, sizeof(TGACompare), plik);	// Odczytanie nag³ówka pliku
		if(memcmp(TGAHeader, TGACompare, sizeof(TGAHeader)) == 0)	// Nag³ówek identyczny?
		{
			fread(Header, 1, sizeof(Header), plik);	// Odczyt u¿ytecznych danych

			// Wy³uskanie informacji o rozmiarach
			info->width  = Header[1] * 256 + Header[0];
			info->height = Header[3] * 256 + Header[2];
			info->bpp = Header[4];

			// Sprawdzenie czy rozmiary > 0 oraz czy bitmapa 24 lub 32-bitowa
			if(info->width>0 && info->height>0 && (info->bpp==24 || info->bpp==32))
			{
				long ImageSize = info->height * info->width * info->bpp / 8;	// Obliczenie iloœci danych
				Bits = (GLubyte*)malloc(ImageSize);	// Alokacja pamiêci na dane

				if(Bits)
				{
					fread(Bits, 1, ImageSize, plik);	// Odczyt w³aœciwych danych pikseli z pliku

					// Konwersja BGR -> RGB
					int i;
					GLubyte tmp;	// Miejsce przechowania jednej wartoœci koloru

					for(i=0;i < ImageSize;i += info->bpp / 8)	// Wszystkie wartoœci RGB lub RGBA
					{
						tmp = Bits[i];
						Bits[i] = Bits[i+2];
						Bits[i+2] = tmp;
					}
				}
			}
    	}

		fclose(plik);
	}

	return(Bits);
}

///////////////////////////////////////////////////////////////////////////////
// Procedury na podstawie przekazanych danych ³aduj¹
// i tworz¹ teksturê lub teksturê z Mip-Map'ami
bool LoadTGATexture(char *filename)
{
	TARGAINFO info;	// Dane o bitmapie
	GLubyte *bits;	// Dane o pikselach

	// ³adowanie pliku
	bits = LoadTGAImage(filename, &info);	// Próba wczytania tekstury
	if(bits == NULL)	return(FALSE);	// ERROR

	// Ustawienie parametrów tekstury
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

	if(info.bpp == 24)	// Bitmapa z danymi RGB
		glTexImage2D(GL_TEXTURE_2D, 0, 3, info.width, info.height, 0, GL_RGB, GL_UNSIGNED_BYTE, bits);
	else	// Bitmapa z danymi RGBA
		glTexImage2D(GL_TEXTURE_2D, 0, 4, info.width, info.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, bits);

	free(bits);
	return(TRUE);
}

bool LoadTGAMipmap(char *filename)
{
	TARGAINFO info;	// Dane o bitmapie
	GLubyte *bits;	// Dane o pikselach

	// ³adowanie pliku
	bits = LoadTGAImage(filename, &info);	// Próba wczytania tekstury
	if(bits == NULL)	return(FALSE);	// ERROR

	// Ustawienie parametrów tekstury
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

	if(info.bpp == 24)	// Bitmapa z danymi RGB
		gluBuild2DMipmaps(GL_TEXTURE_2D, 3, info.width, info.height, GL_RGB, GL_UNSIGNED_BYTE, bits);
	else	// Bitmapa z danymi RGBA
		gluBuild2DMipmaps(GL_TEXTURE_2D, 4, info.width, info.height, GL_RGBA, GL_UNSIGNED_BYTE, bits);

	free(bits);
	return(TRUE);
}
