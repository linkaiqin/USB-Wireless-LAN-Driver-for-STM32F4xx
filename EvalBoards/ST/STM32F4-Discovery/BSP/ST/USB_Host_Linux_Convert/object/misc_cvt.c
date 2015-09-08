#include "string.h"



// /**
//  * strpbrk - Find the first occurrence of a set of characters
//  * @cs: The string to be searched
//  * @ct: The characters to search for
//  */
// char *strpbrk(const char *cs, const char *ct)
// {
// 	const char *sc1, *sc2;

// 	for (sc1 = cs; *sc1 != '\0'; ++sc1) {
// 		for (sc2 = ct; *sc2 != '\0'; ++sc2) {
// 			if (*sc1 == *sc2)
// 				return (char *)sc1;
// 		}
// 	}
// 	return NULL;
// }

/**
 * strsep - Split a string into tokens
 * @s: The string to be searched
 * @ct: The characters to search for
 *
 * strsep() updates @s to point after the token, ready for the next call.
 *
 * It returns empty tokens, too, behaving exactly like the libc function
 * of that name. In fact, it was stolen from glibc2 and de-fancy-fied.
 * Same semantics, slimmer shape. ;)
 */
char *strsep(char **s, const char *ct)
{
	char *sbegin = *s;
	char *end;

	if (sbegin == NULL)
		return NULL;

	end = strpbrk(sbegin, ct);
	if (end)
		*end++ = '\0';
	*s = end;
	return sbegin;
}
