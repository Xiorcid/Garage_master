#include "font.h"
#include "f6x8m.h"
#include "f16f.h"
#include "f24f.h"
#include "f32f.h"

const t_font_getchar font_table_funcs[] = 
{
  f6x8m_GetCharTable,
  f10x16f_GetCharTable,
  f24f_GetCharTable,
  f32f_GetCharTable
};


//==============================================================================
uint8_t *font_GetFontStruct(uint8_t FontID, char Char)
{
  return font_table_funcs[FontID](Char);
}
//==============================================================================


//==============================================================================
uint8_t font_GetCharWidth(uint8_t *pCharTable)
{
  return *pCharTable;
}
//==============================================================================


//==============================================================================
uint8_t font_GetCharHeight(uint8_t *pCharTable)
{
  pCharTable++;
  return *pCharTable;
}
//==============================================================================
